#!/bin/bash
# nav2_errors.sh - count what Nav2 is actually complaining about, in its CURRENT run.
#
# "It worked" and "it worked before too" are both true, and neither settles anything: the failure is
# intermittent. So stop judging it by how it felt and count the thing that fails.
#
# The baseline, from Rodrigo's session on 15-jul (one hour of driving, the camera painting phantom
# obstacles ahead of itself):
#
#     trajectory is not feasible        2809     TEB could not find a way out
#     Controller patience exceeded       162     ...so Nav2 gave up on the goal. THIS is the abort.
#     follow_path Aborting handle        169     -> "I couldn't reach <place>."
#     Collision Ahead - Exiting Spin      22     it could not even turn around to recover
#
# Controller patience exceeded is Nav2 aborting BY ITSELF. Pressing STOP does not produce it. So
# this is the number that says whether the cage is gone -- not whether the last trip felt fine.
#
#   bash ~/ros2_ws/nav2_errors.sh        # since Nav2 started
#
# Drive the same route for the same sort of time, then compare. If the aborts are gone, the cage is
# gone. If they are not, they are not.
# THE LOG OF THE NAV2 THAT IS RUNNING RIGHT NOW, found by its PID -- not the newest file in the
# directory. Those are not the same thing, and the difference silently ruins the measurement: the
# newest log belongs to whichever ROS node last wrote a line, and Nav2's own log is only touched
# when Nav2 has something to say. Relaunch Navigation to reset the counter and `ls -t` happily hands
# you the OLD run's file -- 6620 lines of the failures you just fixed -- and it reads exactly like
# nothing changed. The filename carries the pid: component_container_isolated_<pid>_<stamp>.log.
PID=$(pgrep -f 'component_container_isolated.*nav2_container' | head -1)
[ -z "$PID" ] && PID=$(pgrep -f 'component_container_isolated' | head -1)
if [ -z "$PID" ]; then
    echo "Nav2 is not running -- press the NAVIGATION icon." >&2
    exit 1
fi
L=$(ls -t /home/ubuntu/.ros/log/component_container_isolated_"${PID}"_*.log 2>/dev/null | head -1)
if [ -z "$L" ]; then
    echo "Nav2 (pid $PID) is running but has written no log yet -- give it a moment, or drive." >&2
    exit 1
fi

up=$(( $(date +%s) - $(stat -c %Y "$L") ))
mins=$(awk -v s="$(( $(date +%s) - $(stat -c %W "$L" 2>/dev/null || echo 0) ))" 'BEGIN{print 0}')
started=$(stat -c %y "$L" | cut -d. -f1)

echo "Nav2 log: $(basename "$L")"
echo "started:  $started   ($(wc -l < "$L") lines)"
echo
printf "  %-38s %6s   %s\n" "what Nav2 said" "count" "baseline (15-jul, caged)"
printf "  %-38s %6s   %s\n" "--------------------------------------" "-----" "------------------------"
count() { printf "  %-38s %6d   %s\n" "$2" "$(grep -c "$1" "$L")" "$3"; }
count "trajectory is not feasible"   "TEB: trajectory is not feasible"  "2809"
count "Controller patience exceeded" "ABORTED the goal (patience)"      " 162   <-- THE ONE THAT MATTERS"
count "follow_path\] \[ActionServer\] Aborting" "-> \"I couldn't reach X\""  " 169"
count "Collision Ahead"              "could not even spin to recover"   "  22"
count "failed to create plan"        "global planner found no path"     "  23"
echo
count "Control loop missed its desired rate" "control loop starved of CPU"   " 243"
count "tick rate .* was exceeded"    "behaviour tree starved of CPU"    " 969"
echo
echo "  load now: $(cut -d' ' -f1-3 /proc/loadavg)   (6 cores)"
echo
echo "  The aborts are the verdict. A clean run that FEELS fine proves nothing -- this failed"
echo "  intermittently for days. Drive the same route for the same time, then compare."
