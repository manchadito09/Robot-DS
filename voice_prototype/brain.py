"""
Etapa 3 — CEREBRO 🧠
Toma lo que dijo el visitante y decide: ¿a qué destino quiere ir, y qué le
contesto? Usa el CLI `claude -p` (suscripción Max, SIN API key) — el mismo
patrón que el brain.py de rosita.

Devuelve (destino, frase_hablada). En el robot real, `destino` se traduce a
un POI/objetivo de Nav2 y `frase_hablada` es la narración por el altavoz.
"""

import json
import re
import shutil
import subprocess

# Destinos de la planta 4 (de momento nombres; en el robot real -> coords/POI).
PLACES = {
    "reception": "the reception desk by the lifts",
    "kitchen": "the kitchen and coffee area",
    "meeting room": "the main meeting room",
    "design team": "the design team's desks",
    "restrooms": "the restrooms",
    "exit": "the floor exit",
}

_NAMES = ", ".join(PLACES)

SYSTEM = (
    "You are Wall-E, a friendly indoor guide robot on floor 4 of an office. A "
    "visitor speaks to you and you decide where to take them.\n"
    f"Destinations you can lead to: {_NAMES}.\n"
    "Reply with a SINGLE short spoken sentence (max ~15 words), in the SAME "
    "language the visitor used. Your tone is cheerful, playful and charming, with "
    "a fun and likeable personality -- upbeat and a little witty, never robotic or "
    "dry (but no emojis, since you speak out loud). If the visitor is just greeting or "
    "chatting (not asking to go anywhere), set destination to null AND introduce "
    "yourself by name (say you are Wall-E, the floor 4 guide) before asking where "
    "they want to go.\n"
    'Output ONLY a JSON object, nothing else: '
    '{"destination": <one of the names exactly, or null>, "reply": <spoken sentence>}'
)


def decide(text, lang=None):
    """Pregunta a Claude. Devuelve (destino|None, frase_hablada)."""
    claude = shutil.which("claude")
    if not claude:
        return None, "Sorry, my brain is offline right now."

    prompt = f"{SYSTEM}\n\nVisitor said (language={lang}): \"{text}\""
    try:
        out = subprocess.run(
            [claude, "-p", prompt],
            capture_output=True, text=True, encoding="utf-8", timeout=60,
        )
    except Exception as e:
        return None, f"Sorry, something went wrong. ({e})"

    raw = (out.stdout or "").strip()
    m = re.search(r"\{.*\}", raw, re.DOTALL)
    if not m:
        # Si Claude no devolvió JSON, al menos repetimos lo que dijo.
        return None, raw or "Sorry, I didn't catch that."
    try:
        data = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None, raw

    dest = data.get("destination")
    if isinstance(dest, str) and dest.lower() not in PLACES:
        dest = None
    reply = data.get("reply") or "Okay."
    return dest, reply


if __name__ == "__main__":
    import sys

    # Si escribes tu propia frase, Claude responde a ESA (sin micro).
    #   python voice_prototype/brain.py "quiero imprimir un documento"
    if len(sys.argv) > 1:
        examples = [(" ".join(sys.argv[1:]), None)]
    else:
        # Si no escribes nada, usa unas frases de ejemplo.
        examples = [
            ("I need a coffee", "en"),
            ("¿Dónde está el baño?", "es"),
            ("take me to the design guys", "en"),
            ("hello there", "en"),
        ]

    for t, lg in examples:
        d, r = decide(t, lg)
        print(f'\n  "{t}"  [{lg}]')
        print(f"   -> destino: {d}")
        print(f"   -> dice:    {r}")
