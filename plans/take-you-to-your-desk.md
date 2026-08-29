# "Shall I take you to your desk?" — the robot recognises you and leads you to your desk

Squeezes what we already have: it knows **who** you are (faces) and it knows how to
**lead** you places (waypoints + Nav2). The missing piece is only which waypoint counts
as each person's desk.

## 🔴 Why this is not built from San Francisco

This feature **drives** (it sends a Nav2 goal). To test it you need:

- a human **standing** in front of the robot,
- the battery in and navigation running,
- and to watch it lead someone to a desk without hitting anything.

None of that happens over SSH from another continent. And deploying code that **moves**
the robot while nobody is there to supervise it is a no. **Prepare it now; wire it up and
test it back at the robot** (or with a colleague standing in for you).

## What can be made ready from away

1. **The data**: a `person -> waypoint` map, as JSON in `~/.cache/robot_ds/desks.json`:
   `{ "Rodrigo": "Developers", "Adrian": "Design", ... }`. Editable from the web app —
   in **Add me**, next to each person: *"their desk: [waypoint ▼]"*. Verifiable without
   the robot: that it saves and reads back correctly, and that the waypoint actually
   exists on the map.
2. **The spoken offer** (not driving yet): when it recognises someone who *has* a desk
   saved, instead of just *"Hi Rodrigo!"* it offers *"Hi Rodrigo! Shall I take you to
   your desk?"* and listens for yes/no. The yes/no can be tested; the "yes → drive" part
   is left marked and switched on later.

## What gets wired on return (needs the robot and a person)

3. The "yes" triggers the trip: call navigation (the way the web app does with `/api/go`,
   or via `guide`) to drive to that person's waypoint. Careful — the greeter runs with
   `--no-drive` on purpose. The safe path has to be decided here: does the greeter ask
   the web app / guide to drive? does the greeting stop while it drives? Test it in
   person, with a finger on **STOP**.

## Risks to watch when it is tested

- The robot must **not** set off driving without a clear "yes" — reuse `said_yes`, and
  when in doubt, treat it as no.
- If it is already guiding someone else, it must not abandon them (the same reason the
  chat runs with `--no-drive`).
- The waypoint must exist and be reachable. If it is not, say so out loud rather than
  sitting there doing nothing.

## Order of work

1. *(remote)* `desks.json` + editing it in the web app + validating against the map's
   real waypoints.
2. *(remote)* the spoken offer and the yes/no, **without driving** — just log
   `would drive to X`.
3. *(on return)* switch on the real trip, and test it in person with a hand on STOP.
