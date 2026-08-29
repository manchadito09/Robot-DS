# Operating the robot

Everything you need to run the robot day to day. **No code, no terminal.** If you can
double-click an icon and open a web page, you can run it.

- [Turning it on](#turning-it-on)
- [The web app](#the-web-app)
- [Talking to it](#talking-to-it)
- [Tours](#tours)
- [Teaching it a new place](#teaching-it-a-new-place)
- [Teaching the robot your face](#teaching-the-robot-your-face)
- [The arm](#the-arm)
- [When something goes wrong](#when-something-goes-wrong)
- [Running a demo](#running-a-demo)

---

## Turning it on

**1. Put the battery in.**

Without it there are no servos and no lidar — the robot cannot move its arm or see.
Check it in the web app: **11–12.6 V** means it is on the battery. Around **3.8 V**
means it is *not*, and nothing will work properly.

**2. Double-click `NAVIGATION`** on the desktop.

The brain, the map and the navigation stack come up. Give it a moment.

**3. Double-click `Show QR (phone app)`.**

This one does three jobs: it aims the camera (by moving the arm to a fixed pose),
starts and repairs the whole vision chain, and shows the QR for the phone app.

**4. Wait for the pop-up.** It can take up to ~50 seconds.

```
✅ Robot camera: GREEN     -> ready. It can see chair legs.
⚠️ Robot camera: BLOCKED   -> the lens is covered. Move whatever is in front of it.
🔴 Robot camera: RED       -> the camera did not come up. See "when something goes wrong".
```

> ### 🔴 Never skip GREEN
> ```
> phone shows VIDEO  ─┐
>                     ├── these are NOT the same thing
> robot SEES OBSTACLES┘
> ```
> The lidar sweeps at about 17 cm off the floor, so it flies straight over **chair legs,
> table feet and bin edges**. Only the depth camera sees those, and only if the whole
> chain behind it is alive. `GREEN` is the proof. Without it the robot drives happily
> into a chair while the phone shows a lovely picture. This has cost us a full morning
> more than once.

**5. Open the web app** — scan the QR with a phone, or open it in a browser on the laptop.

---

## The web app

Your remote control. Works from a phone or a laptop, on the same network.

| Section | What it does |
|---|---|
| **Talk to me** | Say or type what you want. The chat shows what the robot says back. |
| **Map** | See the map. Tap a place to go there, save a new one, or remove one. |
| **Tour** | Tick the stops, hit **Start**. Save named tours for later. |
| **Arm** | Wave, Dance, Yes, No, Twirl, Talk mouth, Raise, Look around. |
| **STOP** | Big red button. Stops the wheels *and* the talking. Safe to press any time. |

The status bar shows the important things at a glance: what the robot is doing, the
battery, and the camera state (`ok` / `blocked` / `stale` / `off`).

---

## Talking to it

You can speak (mic) or type. Plain language is fine — it works out what you mean.

**Things that work well:**

```
"Take me to the kitchen"
"Give me a tour"
"Where can I get coffee?"
"What is the glass room?"
"Tell me a joke"
```

**Two habits that keep it smooth:**

| Habit | Why |
|---|---|
| **Be specific.** | "Take me to the kitchen" is unambiguous. "I want to do a lot of things" makes it guess — and it may decide you asked for a whole tour and set off. |
| **One thing at a time.** | Let a trip finish before asking for the next. |

**Descriptions on or off.** There is a switch in the web app. **On**, it tells you about
each place it reaches ("this is the kitchen, where…"). **Off**, it just announces the
arrival — better for people who already know the floor.

---

## Tours

1. Go to **Tour**.
2. Tick the stops you want, in order.
3. Hit **Start**.

It visits each one, explains it, and announces the end. **Let it finish** — interrupting
a tour with questions is the one path that is still rough.

**Saved tours.** Save the ticked stops under a name and it appears as a chip. Tap the
chip to load it, the pencil to rename, the × to delete. A `Default` tour is there from
the start. Saved tours survive restarts.

---

## Teaching it a new place

The robot can only go where it has been shown.

1. Drive or push the robot to the spot (or find the spot on the **Map**).
2. **Map → Add a new point** → tap the floor where the place is.
3. Give it a **name** ("Design team").
4. It asks **what the place is**. This is important — this is what it *says* when it
   arrives, and what it uses to answer questions about it. A suggestion is pre-filled;
   correct it in your own words.
5. Optionally add **other names** people use for it ("design", "the designers").

That is it. The new place is available immediately, by voice and on the map.

---

## Teaching the robot your face

If the robot has the forward camera fitted, an **Add me** button appears on the home screen.
Teach it your face once and it says hello by name when it sees you.

1. Open **Add me**. You will see what the robot sees, with a hint on it: *step in front*,
   *come a bit closer*, *looking good*.
2. Stand about **2 m** in front, so the hint says **Looking good**.
3. Type your name and press **Teach the robot my face**.
4. It takes six photos over about five seconds. **Turn your head a little** between them —
   left, right, up a bit. That is what lets it know you from any angle rather than only in
   the exact pose you posed in.

**No camera handy?** The same screen takes **uploaded photos** — type the name, tap
*Choose photos…*, pick 3–6 clear shots of one face. Scanning works a little better (it uses the
same lens that has to do the recognising) but uploading is easier for a room full of people.
Either way the photos are measured and thrown away.

To make it forget someone, open **Add me** and tap their name.

### Switching the greeting on

The **Say hello to people I know** switch on the same screen. Turn it on **once** — it is
remembered through restarts and reboots, so nobody has to think about it again.

When it recognises someone who stops in front of it, it says one line and goes back to watching:

```
"Hi Rodrigo! Tap Talk to me on the phone app if you need anything."
```

Then it leaves that person alone for five minutes. It only ever *talks* — it will not drive off
or move its arm at you, and it cannot abandon a visitor it is already leading.

Someone it does **not** know gets a different opening — *"Hello! I'm Wall-E, the floor guide.
Scan the QR code on me and tap Talk to me, and I'll take you where you need to go."*

> **It says hello; it does not listen.** Answer it out loud and nothing happens — that is not a
> fault. The robot's own microphone is far-field and sits next to its speaker, which never worked
> reliably, so talking to it lives in the app under **Talk to me**, where your phone does the
> listening. Every greeting says so out loud. The greeter never opens a microphone at all.

> **What is stored.** A *fingerprint* — 128 numbers — and never a photo. Your face cannot be
> rebuilt from it, and no picture of you is kept anywhere on the robot. Enrolment is opt-in by
> construction: someone has to stand here and choose to do it.
>
> It is still biometric data about employees. Before enrolling a floor of people, get a clear
> yes from whoever owns privacy at the company. That is a conversation, not a checkbox.

How it works underneath — the two cameras, the recognition, the self-learning locks — is in
[faces.md](./faces.md).

---

## The arm

The arm is for **expression**, not for picking things up. Every gesture is a button —
the robot never moves the arm at you on its own.

| Button | What it does |
|---|---|
| **Wave** | Raises the arm and waves — a proper hello. |
| **Big wave** | Arm up high, swinging side to side. |
| **Raise** | Reaches up and holds — "presenting". |
| **Yes** / **No** | Nods / shakes. |
| **Dance** | A silly little shimmy. |
| **Twirl** | Spins the hand. |
| **Talk mouth** | Opens and closes the gripper like a mouth. |
| **Look around** | Turns one way, then the other. |

Gestures do not upset navigation: the robot re-aims the camera automatically before
every trip.

---

## When something goes wrong

### The robot bumps into chair legs

**The vision chain is down.** The camera may well be showing video — that is not the same
thing. Double-click **Show QR (phone app)** and wait for `GREEN`.

### Camera says BLOCKED

The camera is healthy, the *view* is not. Something is in front of the lens — a hand, a
bag, a wall it is parked against. Move it. It recovers on its own.

### Camera says RED, or the status shows `stale` / `off`

1. Wait ~30 seconds. The robot notices and repairs the camera by itself.
2. Still bad? Double-click **Show QR (phone app)** again.
3. Still bad? Restart the robot (cold), then `NAVIGATION` → `Show QR`.
4. Still bad and you are out of time? **You can still run without it** — see
   [running without the camera](#running-without-the-camera).

### It stops in the middle of a trip and says the camera is gone

That is the safety watchdog doing its job: without the depth camera it is blind to low
obstacles, so it stops instead of driving on blind. It repairs the camera and waits for
you to ask again. It will not set off on its own.

### It freezes in an empty corridor

It thinks it sees obstacles that are not there. Almost always the arm is not in its
camera pose — usually because the **battery** is out, so the servos never moved. Put the
battery in and press **Show QR** again.

### It will not go somewhere

- Say the place name clearly, or tap it on the **Map** instead.
- Check it is where it thinks it is (the red dot on the map). If not, use **I'm here**.
- Check `NAVIGATION` is actually running.

### The web app looks odd

Reload the page. The page is just a remote control — reloading never affects the robot.

### It is about to hit something

**STOP.** Big red button, always on screen.

### It greets no one, or the wrong name

- Check the greeting switch is on (**"Say hello to people I know"**).
- Greets no one: the face camera may be unplugged or the arm is not in its scan pose.
- Wrong name or wobbly from afar: that person was added from flat photos — re-add them
  by **scanning** and turning their head. More in [faces.md](./faces.md#health-and-troubleshooting).

---

## Running a demo

A tested running order that shows the robot at its best.

**Before (allow 10 minutes):**

```
✓ battery in and charged
✓ NAVIGATION
✓ Show QR  ->  wait for GREEN            <- do not skip
✓ one full rehearsal trip, start to finish
✓ robot parked at base, correctly placed on the map
✓ descriptions ON
✓ one obstacle placed on the route
✓ web app open, camera shows ok
```

**The run:**

```
1. VOICE:    "Take me to the kitchen"   -> drives, narrates, avoids, arrives
2. STOPPED:  "What is the glass room?"  -> answers
3. ARM:      Wave -> Dance              -> personality
4. FINALE:   "Give me a tour"           -> let it finish
```

**While it drives, the things worth saying out loud:** no teleoperation — it decides and
navigates by itself; it takes plain language, by voice or screen; it knows the building
and the teams.

### Running without the camera

If the camera will not come up and you are out of time, the demo still works — the lidar
does the driving. The camera only adds *low* obstacles.

1. **Clear the route of low things**: chairs, table feet, bins, cables.
2. **Use a tall obstacle** to show off avoidance: a cardboard box, a backpack, a person.
   Anything over ~17 cm is squarely in the lidar's view.
3. Ask someone technical to disable the camera watchdog first — otherwise the robot stops
   a few seconds into every trip, on purpose (see [`developing.md`](./developing.md)).

Nobody watching can tell the difference.

---

## Known rough edges

Honest list, as of the first live demo:

| Rough edge | Workaround |
|---|---|
| **Questions asked while it drives a tour** sometimes get no answer at all. | Ask while it is stopped. |
| **Vague requests** ("I want to do a lot of things") can make it start a whole tour by itself. | Be specific. |
| A **messenger message** was not delivered when descriptions were switched off. | Fixed; needs a rebuild to take effect. |
| If it dies mid-trip, the camera watchdog **stopping the trip** is coded but has not been confirmed live on battery. | Keep an eye on it during trips. |
