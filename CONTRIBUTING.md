# Contributing — got an idea for the robot?

Good. This robot exists to be poked at. You do **not** need to know ROS, or robotics,
or even how to code, to help it get better. This page is how.

---

## Just have an idea? (no code)

You don't have to build anything. If you've watched the robot and thought *"it should
do X"*, that idea is worth writing down — someone who codes can pick it up later.

Leave it as a **GitHub issue**. An "issue" is just a note on the project's page — think
of it as a **suggestions box**. You type your idea in plain words (no code), and it stays
there for someone to act on. To leave one:

> Go to **[github.com/manchado53/Robot-DS](https://github.com/manchado53/Robot-DS)** →
> the **Issues** tab → **New issue** → write it and submit.

Say what you saw, in three lines:

```
What should it do?        "It should say goodbye when you leave, not just hello."
Why would that help?      "Right now it goes quiet and people think it broke."
Where did you see it?     "Front desk, when visitors walk off mid-tour."
```

That's a real contribution. Half of the best changes here started as one sentence from
someone who was just watching the robot work.

> **Not comfortable with GitHub? Just tell Rodrigo.** The whole point is that a good
> idea doesn't get lost — the channel matters far less than the idea.

---

## Want to build it yourself?

First, get the code:

```bash
git clone https://github.com/manchado53/Robot-DS.git
```

(On the robot itself the code is already there — see the three-copies note below.) Then
three habits keep this codebase sane. They are not bureaucracy — each one is here because
skipping it cost someone a day.

### 1. One branch per idea. `main` is sacred.

Never commit to `main`. Branch first, name it for the intent:

```bash
git checkout main
git checkout -b feat/say-goodbye        # feat/  fix/  docs/  experiment/
```

Work on the branch. Come back to `main` only through a reviewed pull request.

### 2. Know which copy you are editing.

There are **three copies of the source** on the robot and one is a dead fossil. Edit
the wrong one and your change does nothing, silently, forever. Before you touch any
code, read the first two sections of **[docs/developing.md](./docs/developing.md)**.
The short version:

```
~/ros2_ws/          ← the robot runs from here. Edit here.
~/robot-ds-clone/   ← git lives here. Commit from here.
~/Robot-DS/robot_web/  ← the web app runs from here.
```

And `guide.py` / `brain.py` run from a **build**, not the file you saved —
`colcon build` after editing them, or the robot keeps running the old code.

### 3. Measure, don't assume.

The hardest bugs here were never where they looked. A "camera bug" was a busy-wait in a
maths library; a "navigation bug" was two noise filters erasing a chair leg. Before you
"fix" something, prove what it actually does — there's a whole shelf of diagnostic tools
for exactly this in [docs/developing.md](./docs/developing.md#diagnostic-tools).

---

## The one rule that is not negotiable

> **No `GREEN`, no go.** The phone showing video does **not** mean the robot can see
> obstacles. `Robot camera: GREEN` is the only proof the whole vision chain is alive.
> Never start a demo — or a test that drives — without it.

---

## Where everything is

| You want to… | Read |
|---|---|
| Use the robot (no code) | [docs/operating.md](./docs/operating.md) |
| Change the robot | [docs/developing.md](./docs/developing.md) |
| Understand faces / greeting | [docs/faces.md](./docs/faces.md) |
| Understand the voice half | [docs/voice-guide.md](./docs/voice-guide.md) |
| Understand mapping & driving | [docs/movement-nav.md](./docs/movement-nav.md) |
| See what's done / what's next | [plan.md](./plan.md) |

---

## Sending your change

1. Push your branch and open a **pull request** against `main`.
2. Say **what** it does and **how you tested it** — "drove it past a chair, it went
   around" beats "should work".

That's it. Small and clear beats big and clever. Welcome aboard.
