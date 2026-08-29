#!/usr/bin/env python3
# brain.py - puts Claude in front of the guide node.
# A free-form sentence -> Claude picks the POI -> guide.py drives there with Nav2,
# and on arrival the robot NARRATES what the place is (varied wording each time).
#
# It reads office_knowledge.yaml (the office "knowledge base") so Claude can:
#   (1) pick the destination smarter, using each place's ALIASES + what it is,
#   (2) NARRATE on arrival with a short, natural, always-slightly-different line,
#   (3) ANSWER questions about a place (answer_question(), ready but not wired yet).
#
# `pick_poi()` still lives here and keeps the SAME signature (talk.py imports it).
# Uses the `claude -p` CLI (Max subscription, no API key). If Claude isn't
# available, every function degrades to a safe, deterministic fallback so the
# robot never goes silent.
#
# Usage (with Nav2 running):
#     python3 brain.py "i'm hungry"            # lead there + return to base
#     python3 brain.py --stay "i'm hungry"     # lead there and stay (voice loop)
import os
import time as _time
_T0 = _time.time()


def _lap(what):
    """Timestamps for the voice path. Every 'why is the robot so slow' answer so far has been
    somewhere nobody was looking, so print where the seconds actually go: ROBOT_TIMING=1."""
    if os.environ.get("ROBOT_TIMING"):
        print(f"[t+{_time.time()-_T0:5.2f}s] {what}", flush=True)
import sys
import time
import shutil
import subprocess
import yaml
import rclpy
try:  # installed package: ros2 run (the real robot, Humble)
    from robot_ds_behavior.guide import DEFER_LINE, Guide, load_places, say
except ImportError:  # loose scripts: python3 brain.py (the sim on rosita)
    from guide import DEFER_LINE, Guide, load_places, say

# The office knowledge base: what each place is, its aliases, who's there, etc.
KNOWLEDGE_YAML = os.path.expanduser("~/ros2_ws/office_knowledge.yaml")

# Where we remember a PENDING OFFER between utterances. The robot is launched
# fresh for every sentence (voice/web run `brain --stay "<text>"`), so it can't
# hold memory in RAM. When it offers ("want me to take you to the kitchen?") we
# jot the place here; the next sentence, a "yes" gets turned into a real GO.
PENDING_FILE = os.path.expanduser("~/.cache/robot_ds/pending_offer.txt")
PENDING_MAX_AGE = 120.0    # seconds; an older offer is stale and ignored


# ---- knowledge base ---------------------------------------------------------
def load_knowledge(path=KNOWLEDGE_YAML):
    """Read office_knowledge.yaml -> dict with 'office' and 'places'.
    Empty ({}) if the file is missing or unreadable, so callers still work."""
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (FileNotFoundError, OSError):
        return {}


def _clean(value):
    """A knowledge field is only usable if it's non-empty and not a 'TODO' note."""
    if not value:
        return ""
    text = str(value).strip()
    if text.upper().startswith("TODO"):
        return ""
    return text


def knowledge_for(name, kb=None):
    """The knowledge entry for one place name, or {} if there's none."""
    kb = kb if kb is not None else load_knowledge()
    return (kb.get("places") or {}).get(name, {}) or {}


def _facts_line(name, place):
    """Join a place's usable facts into one plain sentence-ish string (for Claude
    prompts and as the fallback narration). Only real, non-TODO fields."""
    parts = []
    for key in ("what", "who", "fun_fact"):
        val = _clean(place.get(key))
        if val:
            parts.append(val)
    return " ".join(parts) if parts else f"This is {name}."


# ---- pending offer memory (between utterances) ------------------------------
def _save_pending(name):
    """Remember 'I just offered to take them to <name>' with a timestamp."""
    try:
        os.makedirs(os.path.dirname(PENDING_FILE), exist_ok=True)
        with open(PENDING_FILE, "w") as f:
            f.write(f"{name}\t{time.time()}")
    except OSError:
        pass


def _load_pending():
    """The place we last offered, if the offer is still fresh; else None.
    Stale/absent offers return None so an old 'yes' can't trigger a surprise trip."""
    try:
        with open(PENDING_FILE) as f:
            name, _, ts = f.read().partition("\t")
    except (FileNotFoundError, OSError):
        return None
    name = name.strip()
    if not name:
        return None
    try:
        if time.time() - float(ts) > PENDING_MAX_AGE:
            return None
    except ValueError:
        return None
    return name


def _clear_pending():
    """Forget any pending offer (after we act on it or move on to something else)."""
    try:
        os.remove(PENDING_FILE)
    except OSError:
        pass


# ---- Claude helper ----------------------------------------------------------
# FAST PATH: a warm `claude` daemon (claude_daemon.py) keeps one process alive and
# answers over a unix socket in ~2s, vs ~10s to cold-start `claude -p` every call.
# It sends /clear before each prompt, so answers stay INDEPENDENT (same behaviour as
# the old one-shot call). If the daemon is down or errors, we fall back to plain
# `claude -p` -- the robot never breaks, it just gets slower.
import socket as _socket
import struct as _struct

CLAUDE_SOCK = "/tmp/robot_ds_claude.sock"


def _ask_daemon(prompt, timeout=50):
    """Ask the warm-claude daemon. Returns the reply, or None if it's unreachable/errored
    (caller then falls back to `claude -p`)."""
    try:
        s = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(CLAUDE_SOCK)
        data = prompt.encode("utf-8")
        s.sendall(_struct.pack(">I", len(data)) + data)
        raw = s.recv(4)
        if len(raw) < 4:
            return None
        n = _struct.unpack(">I", raw)[0]
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                break
            buf += chunk
        s.close()
        reply = buf.decode("utf-8").strip()
        return reply or None          # empty reply => daemon failed => fall back
    except Exception:
        return None


def _ask_claude(prompt, timeout=60):
    """Send a prompt to Claude. Tries the warm daemon first (fast), falls back to the
    `claude -p` CLI (slow but always works). Returns the stripped reply, or ''."""
    _lap(f"claude: asking ({len(prompt)} chars)")
    fast = _ask_daemon(prompt)
    if fast is not None:
        _lap("claude: answered via the WARM DAEMON")
        return fast
    _lap("claude: daemon did not answer -> COLD `claude -p` (this is the ~10s path)")
    claude = shutil.which("claude")
    if not claude:
        return ""
    try:
        out = subprocess.run([claude, "-p", prompt],
                             capture_output=True, text=True, timeout=timeout)
        return (out.stdout or "").strip()
    except Exception:
        return ""


def _places_block(places, kb, full=False):
    """One line per place for Claude prompts: "'name' | also called: ... | what".
    Bare name if there's no knowledge entry. Shared by pick_poi() and route().
    full=True also appends who/fun_fact, so ONE route() call can both pick the
    place AND narrate it richly on arrival (saves a whole second Claude call)."""
    lines = []
    for n in places:
        place = knowledge_for(n, kb)
        aka = ", ".join(str(a) for a in (place.get("aka") or []))
        what = _clean(place.get("what"))
        bits = [f"'{n}'"]
        if aka:
            bits.append(f"also called: {aka}")
        if what:
            bits.append(what)
        if full:
            who = _clean(place.get("who"))
            fun = _clean(place.get("fun_fact"))
            if who:
                bits.append("who: " + who)
            if fun:
                bits.append("fun fact: " + fun)
        lines.append("- " + " | ".join(bits))
    return "\n".join(lines)


def _match_place(answer, places):
    """Map Claude's free reply to an exact waypoint name: exact match wins, else
    the first place name that appears inside the reply. None if nothing matches."""
    answer = (answer or "").lower()
    if answer in places:
        return answer
    for n in places:
        if n.lower() in answer:
            return n
    return None


# ---- (1) pick the destination ----------------------------------------------
def pick_poi(text):
    """Use Claude to turn a free-form sentence into an exact destination name.
    Destinations are the named waypoints you saved while mapping (load_places).
    Now enriched with office_knowledge.yaml: Claude sees each place's ALIASES and
    what it is, so it matches indirect requests ('coffee', 'a quiet call') much
    better. Returns the exact waypoint name or None."""
    places = load_places()
    kb = load_knowledge()
    claude = shutil.which("claude")
    if not claude:
        # no Claude: last resort, match a literal name OR a known alias in the text.
        low = text.lower()
        for n in places:
            if n.lower() in low:
                return n
        for n in places:
            for alias in (knowledge_for(n, kb).get("aka") or []):
                if str(alias).lower() in low:
                    return n
        return None
    prompt = (
        "You are the brain of a guide robot that leads visitors to places. "
        "Here are the places you can go to, with their aliases and what they are:\n"
        + _places_block(places, kb) + "\n"
        "The visitor talks naturally and often indirectly -- they may describe a "
        "need, an activity, or a feeling instead of naming a place. Reason about "
        "what each place is for and choose the ONE place that best matches what "
        "they actually want. "
        "Answer 'none' only if it is a greeting, small talk, or has no sensible "
        "connection to any of the places. "
        "Answer with ONLY the exact place name (the word in quotes above) in "
        "lowercase, or 'none'. "
        "The visitor says: " + text
    )
    answer = _ask_claude(prompt).lower()
    if not answer or "none" in answer.split():   # Claude bailed out on purpose
        return None
    return _match_place(answer, places)


# ---- (2) narrate on arrival -------------------------------------------------
def describe(name):
    """A short, spoken line about a place, said when the robot arrives.
    Claude writes it fresh (same idea, slightly different words every time); if
    Claude isn't available it falls back to the plain facts from the yaml, so
    the robot always says SOMETHING. Returns a string."""
    place = knowledge_for(name)
    facts = _facts_line(name, place)             # deterministic fallback text
    prompt = (
        "You are a friendly office guide robot that has just arrived at a place "
        "with a visitor. In ONE or at most TWO short spoken sentences, tell them "
        "about it in a warm, natural way. Vary your wording so it doesn't sound "
        "scripted. Always speak in ENGLISH. Plain text only: no quotes, markdown, "
        "emojis or lists. "
        "Facts about this place: " + facts + " "
        "Say your line now:"
    )
    reply = _ask_claude(prompt, timeout=30)
    return reply or facts


# ---- (3) answer a question about a place (ready, not wired to the robot yet) -
def answer_question(text):
    """Answer a visitor's question about the office/its places using the
    knowledge base. Spoken, concise. Returns a string. NOT wired into the robot
    flow yet -- call it from a script to test it cold."""
    kb = load_knowledge()
    office = kb.get("office") or {}
    lines = []
    for n, place in (kb.get("places") or {}).items():
        aka = ", ".join(str(a) for a in (place.get("aka") or []))
        facts = _facts_line(n, place)
        head = f"{n}" + (f" (also: {aka})" if aka else "")
        lines.append(f"- {head}: {facts}")
    prompt = (
        "You are a friendly office guide robot for "
        + str(office.get("company", "this office")) + " on the "
        + str(office.get("floor", "office")) + ". "
        "A visitor asks you a question. Answer it in one or two short spoken "
        "sentences using ONLY the facts below. If the answer isn't in the facts, "
        "say you don't know but offer to take them somewhere. Always answer in "
        "ENGLISH. Plain text only.\n"
        "Places you know:\n" + "\n".join(lines) + "\n"
        "The visitor asks: " + text
    )
    reply = _ask_claude(prompt, timeout=30)
    return reply or "Sorry, I'm not sure about that one. I can take you somewhere if you like."


def _parse_route(reply):
    """Parse the router's answer into (action, place_raw, say_text, narrate).
    Format (each field ONE line):  ACTION: ...  /  PLACE: <name(s) or ->  /
    REPLY: <spoken text for 'say'/'tour'>  /  NARRATE: <arrival line for 'go'>.
    PLACE is left RAW (route() matches it). Lenient: missing fields -> safe 'none'."""
    f = {"action": "none", "place": "", "reply": None, "narrate": None}
    for ln in reply.splitlines():
        low = ln.strip().lower()
        for key in ("action", "place", "reply", "narrate"):
            if low.startswith(key + ":"):
                f[key] = ln.split(":", 1)[1].strip()
                break
    return f["action"].lower(), f["place"], f["reply"], f["narrate"]


def _match_place_list(raw, places):
    """A comma-separated PLACE string -> ordered list of valid waypoint names
    (drops anything that doesn't match, keeps order, no duplicates)."""
    out = []
    for chunk in raw.split(","):
        name = _match_place(chunk.strip().lower(), places)
        if name and name not in out:
            out.append(name)
    return out


def _default_tour(kb, places):
    """The fallback tour when the visitor asks for one without naming stops:
    the office's demo_places (filtered to real waypoints), else all places."""
    demo = [n for n in (kb.get("office", {}).get("demo_places") or []) if n in places]
    return demo or list(places)


# ---- the router: one sentence -> what should the robot DO? -------------------
def route(text, pending=None):
    """Decide, in ONE Claude call, what the robot should do with a sentence:
        ("go",   name, None)     -> drive to that waypoint and narrate on arrival
        ("tour", [names], intro) -> guided tour: visit each stop in order, narrating;
                                    `intro` is a friendly line to say before starting
        ("say",  place, reply)   -> speak this answer; if `place` is set it means we
                                    OFFERED to take them there (remember it, so a
                                    later 'yes' becomes a GO). Robot does NOT move.
        ("none", None, None)     -> greeting / small talk / unclear
    `pending` = the place we offered on the previous turn (from _load_pending);
    a confirmation ('yes', 'sure') then turns into a GO there.
    Falls back to pick_poi() (go-or-nothing) when Claude isn't available."""
    places = load_places()
    kb = load_knowledge()
    if not shutil.which("claude"):
        name = pick_poi(text)                     # no Claude: can only try to go
        return ("go", name, None) if name else ("none", None, None)
    office = kb.get("office") or {}
    context = ""
    if pending:
        context = ("Context: a moment ago you offered to take them to '" + pending
                   + "'. If they now agree or confirm (yes, sure, okay, please do, "
                   "let's go), choose action 'go' with that place.\n")
    default_tour = ", ".join(_default_tour(kb, places))
    # What the robot knows about the COMPANY, not just the floor plan. Without it, "what is Direct
    # Supply?" got "I'm just the guide robot, I don't know much about the company" -- which was
    # honest, and useless. The facts live in office_knowledge.yaml (office.about) so a human writes
    # them: this thing talks in front of the real company, and an invented fact that sounds right is
    # far worse than an admitted gap.
    # about + about_extra. `about` is also the pool guide.py fills a corridor from; `about_extra` is
    # answer-only -- true and worth knowing, but not what you say to someone you are walking down a
    # hallway (three product names in one breath is a brochure). Claude gets both: asked "what do
    # you actually sell?", the robot must not have to admit a gap it does not have.
    about = [str(a).strip()
             for a in ((office.get("about") or []) + (office.get("about_extra") or []))
             if str(a).strip()]
    about_block = ("About the company (use these facts, do not invent others):\n"
                   + "\n".join("- " + a for a in about) + "\n") if about else ""
    prompt = (
        "You are the brain of a friendly office guide robot for "
        + str(office.get("company", "this office")) + " on the "
        + str(office.get("floor", "office")) + ". A visitor speaks to you.\n"
        + about_block +
        "Places you can physically take them to (use these exact names):\n"
        + _places_block(places, kb, full=True) + "\n"
        + context +
        # BE BRIEF, and mean it. Without this, REPLY came back at 40 words -- which is THIRTEEN
        # SECONDS of a robot talking at someone who asked a five-second question, and Piper slurs
        # and swallows words on lines that long. Speech is not text: every extra word is another
        # second of a visitor standing there. Measured: the reply took 13 s to say and 3.3 s to
        # think about.
        "SPEAK LIKE A PERSON, NOT A BROCHURE. Every line you write is SPOKEN OUT LOUD by a robot "
        "standing in front of someone. REPLY and NARRATE must each be AT MOST 15 WORDS -- ONE short "
        "sentence. No lists, no rambling, no repeating the question back. If you have more to say, "
        "say the best half.\n"
        "Decide ONE action and answer with EXACTLY these four lines (each ONE line):\n"
        "ACTION: go | tour | say | none\n"
        "PLACE: the exact place name(s) involved, or - if none\n"
        "REPLY: what to say out loud, MAX 15 WORDS (for 'say', or a short intro for 'tour'; else -)\n"
        # The robot ALREADY says "We've arrived at <place>." and this line is spoken straight after
        # it, in the same breath. Claude kept restating the arrival on top of it -- "We've arrived at
        # Kitchen. And here we are at the kitchen, the spot where..." -- which is the robot saying
        # the same thing twice in one sentence. Tell it the arrival is handled, and to just describe
        # the place.
        "NARRATE: for 'go' ONLY, MAX 15 WORDS describing the destination, to say on arrival "
        "(vary the wording, use the facts above). The robot ALREADY says \"We've arrived at "
        "<place>.\" immediately before this line, so do NOT announce the arrival again: no "
        "\"we've arrived\", no \"here we are\", no \"welcome to\". Start straight in on what the "
        "place IS. Otherwise -\n"
        "How to choose:\n"
        "- go: they clearly ask to be TAKEN to ONE place now, or confirm a previous "
        "offer. PLACE = the single destination. Fill NARRATE for arrival.\n"
        "- tour: they ask for a TOUR, to be shown around, or to visit SEVERAL places. "
        "PLACE = the exact stop names in visit order, comma-separated. If they don't "
        "name places, use this default set: " + default_tour + ". REPLY = one warm "
        "line introducing the tour (you may name the stops).\n"
        "- say: they ASK A QUESTION, express a need/wish/feeling that maps to a "
        "place, OR it is a greeting, thanks, small talk, or a polite 'no'. Always "
        "respond warmly in ONE friendly line in REPLY (using only the facts above "
        "for questions). If it maps to a place, put it in PLACE and end REPLY by "
        "offering to take them there; otherwise PLACE -. If you don't know an "
        "answer, say so and offer to take them somewhere.\n"
        "- none: ONLY when the speech is empty, garbled, or truly unintelligible "
        "(PLACE -, REPLY -).\n"
        "Always write REPLY in ENGLISH, whatever language the visitor used.\n"
        "The visitor says: " + text
    )
    reply = _ask_claude(prompt).strip()
    action, place_raw, say_text, narrate = _parse_route(reply)
    narrate = None if (not narrate or narrate == "-") else narrate
    if action == "tour":
        names = _match_place_list(place_raw, places) or _default_tour(kb, places)
        intro = say_text if (say_text and say_text != "-") else "Sure, let me show you around!"
        return ("tour", names, intro)
    if action == "go":
        name = None if place_raw in ("", "-") else _match_place(place_raw.lower(), places)
        # narrate came from the SAME call -> no separate describe() call (half the wait)
        return ("go", name, narrate) if name else ("none", None, None)
    if action == "say" and say_text:
        place = None if place_raw in ("", "-") else _match_place(place_raw.lower(), places)
        return ("say", place, say_text)
    return ("none", None, None)


def main():
    # --stay is kept for the caller's contract (voice/web pass it); return-to-base
    # is disabled anyway, so the robot always stays where it's sent.
    #
    # --no-drive: ANSWER, NEVER MOVE. This is how the robot holds a conversation while it is already
    # walking someone somewhere. The web runs a second brain alongside the trip, and that second
    # brain must not be able to send a Nav2 goal: two goals at once and the robot would abandon the
    # visitor it is leading, mid-corridor, to go somewhere else. Talking is safe to do twice at
    # once; driving is not.
    no_drive = "--no-drive" in sys.argv
    words = [a for a in sys.argv[1:] if not a.startswith("--")]
    text = " ".join(words).strip()
    if not text:
        return
    _lap("process up, imports done")
    action, place, reply = route(text, pending=_load_pending())
    _lap(f"route decided: {action}")

    if no_drive and action in ("go", "tour"):
        # They asked to be taken somewhere while we are already taking them somewhere. Do not
        # silently ignore it -- being asked and answered with nothing is the rudest thing a robot
        # can do. Say we heard, and say when we can. Do NOT stash it as a pending offer: an offer
        # answered "yes" ten minutes later, having arrived somewhere else entirely, is a robot that
        # sets off on its own.
        _clear_pending()
        _lap("asked to move mid-trip -> deferring, not driving")
        say(DEFER_LINE)          # pre-baked (guide.NARRATE_LINES): no Piper while driving
        return

    if action == "say":
        # A question or an offer ("want me to take you?"): speak, don't move.
        # If we offered a place, remember it so a later "yes" becomes a real trip.
        if place:
            _save_pending(place)
        else:
            _clear_pending()
        _lap("speaking (Piper generates the audio, THEN plays it)")
        say(reply)
        _lap("done speaking")
        return
    if action not in ("go", "tour"):
        _clear_pending()
        # Not understood: say so out loud and ask to repeat -- never guess a goal.
        say("Sorry, I didn't get that. Could you say it again?")
        return
    _clear_pending()                     # we're acting on it; the offer is spent
    rclpy.init()
    node = Guide()
    try:
        # Return-to-base is disabled (guide.base_name() is None), so we use go()
        # directly and pass the arrival narration.
        if action == "tour":
            names = place                # a list of stops, in order
            print("Tour:", names)
            say(reply)                   # friendly intro before we set off
            for n in names:
                node.go(n, arrive_say=describe(n))
            say("That's the end of the tour. Thanks for coming along!")
        else:
            name = place
            print("Claude picked:", name)
            # `reply` is the arrival narration from the SAME route() call (no extra
            # Claude call). Fall back to describe() only if it came back empty.
            node.go(name, arrive_say=(reply or describe(name)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
