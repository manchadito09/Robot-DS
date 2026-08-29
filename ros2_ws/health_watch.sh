#!/bin/bash
# health_watch.sh - run this DURING a rehearsal. It writes one line every 10 s.
#
# "After a while the robot starts doing odd things." A fault that gets WORSE WITH TIME is not a
# planner bug -- Nav2 does not get tired. Something is ACCUMULATING. We have seen exactly that
# before: a false camera repair, fired every 60 s, leaving orphans behind each time, until FOUR
# camera drivers were fighting over one USB device and TWO camera_scan_nodes were feeding the
# costmap contradictory scans. The robot went, in Rodrigo's words, mad.
#
# That accumulator is fixed. This watches for the next one. It records the four things that would
# show it -- and it records them over TIME, because a single snapshot of a healthy-looking robot is
# what let this hide for days:
#
#   drivers   there must be exactly ONE. Two is the bug coming back.
#   scan node there must be exactly ONE. Two = contradictory scans in one costmap.
#   load      this Jetson has 6 cores. Past ~6 everything starves: Whisper 8 s, Nav2 in lurches.
#             CPU is the first thing to check, always, before you optimise anything.
#   camera    ok | blocked (fine, something is in front) | stale/off (actually broken)
#
#   bash ~/ros2_ws/health_watch.sh              # -> /tmp/health_watch.log, and to the screen
#   bash ~/ros2_ws/health_watch.sh --quiet      # log only
#
# Ctrl-C to stop. Afterwards: look at the line where the robot started misbehaving, and the ten
# before it.
LOG=/tmp/health_watch.log
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

# Anchored patterns: pgrep -f matches whole command lines, and several things here carry these names
# inside their own pgrep. Match the bare name and we count someone else's SEARCH as a process.
#
# THE CAMERA DRIVER, and only it. The old pattern was 'component_container' -- which is a PREFIX of
# 'component_container_isolated', the process Nav2 runs. So it counted the camera driver AND Nav2 and
# reported "2 DRIVERS -- the accumulator is back" on every single line, all night, for a robot that
# had exactly one of each. A false alarm that cried wolf so constantly it drowned out the real ones.
# The camera driver is the component_container whose node is camera_container, on the depth_cam
# namespace -- match that, and nothing else can look like it.
DRIVER='component_container .*camera_container'
SCAN='^python3 .*camera_scan_node\.py'

hdr="        time  drv  scan   load  battery  camera    note"
echo "# health_watch started $(date)" >>"$LOG"
echo "$hdr" | tee -a "$LOG"

while true; do
    # pgrep -c already prints 0 when nothing matches; the old `|| echo 0` ADDED a second 0 on the
    # miss, so drv became "0\n0" and every [ ] test below threw "integer expression expected".
    drv=$(pgrep -cf "$DRIVER" 2>/dev/null); drv=${drv:-0}
    scan=$(pgrep -cf "$SCAN" 2>/dev/null); scan=${scan:-0}
    load=$(cut -d' ' -f1 /proc/loadavg)
    S=$(curl -s -m 3 http://127.0.0.1:8000/api/status 2>/dev/null)
    cam=$(echo "$S" | grep -o '"camera": "[a-z]*"' | cut -d'"' -f4)
    bat=$(echo "$S" | grep -o '"battery_v": [0-9.]*' | cut -d' ' -f2)

    note=""
    [ "$drv"  -gt 1 ] && note="$note  !! ${drv} DRIVERS -- the accumulator is back"
    [ "$scan" -gt 1 ] && note="$note  !! ${scan} SCAN NODES -- two /camera_scan into one costmap"
    [ "$drv"  -eq 0 ] && note="$note  !! NO camera driver"
    # 6 cores. Past 6 the robot is starving, and everything looks like four separate bugs at once.
    awk -v l="$load" 'BEGIN{exit !(l>6.0)}' && note="$note  !! LOAD ${load} -- no CPU left"

    # WHO is eating it. A load alarm without a name sent us hunting for three days: the answer was
    # one Python process burning 374% of the CPU to do 29% of the work, because numpy had started
    # six OpenBLAS threads and left them BUSY-WAITING. Busy-waiting looks exactly like being busy.
    # So do not just say the machine is loaded -- say who, and by how much. Anything over 150% of a
    # core on this robot is either genuinely hard work or a thread pool spinning, and both are worth
    # a name in the log at the moment it starts.
    hog=$(ps -eo pcpu=,comm=,pid= --sort=-pcpu | awk '$1 > 150 {printf "%s(%s) %s%% ", $2, $3, $1}')
    [ -n "$hog" ] && note="$note  !! EATING THE CPU: $hog"
    case "$cam" in
        stale|off) note="$note  !! camera ${cam} (actually broken, not just covered)" ;;
    esac
    # The battery reads three ways, and they mean three different things:
    #   11-12.6 V  fine
    #   5-11 V     weak motors, the robot staggers -- and from here on you are measuring the
    #              battery, not the robot. Nothing you observe below 11 V is worth trusting.
    #   ~3.8 V     there is NO battery: it is on the Jetson cable. No servos, no arm, no driving.
    if [ -n "$bat" ]; then
        awk -v b="$bat" 'BEGIN{exit !(b<11.0 && b>5.0)}' \
            && note="$note  !! BATTERY ${bat}V -- weak motors, measure nothing until you charge it"
        awk -v b="$bat" 'BEGIN{exit !(b<5.0)}' \
            && note="$note  (no battery -- on the cable, it cannot drive)"
    fi

    line=$(printf "%12s  %3s  %4s  %5s  %6sV  %-8s%s" \
        "$(date +%H:%M:%S)" "$drv" "$scan" "$load" "${bat:-?}" "${cam:-?}" "$note")
    echo "$line" >>"$LOG"
    [ "$QUIET" = "0" ] && echo "$line"
    sleep 10
done
