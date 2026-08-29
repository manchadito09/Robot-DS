#!/bin/bash
# speaker.sh - mute the robot ON PURPOSE, and make sure it cannot stay muted by accident.
#
#   bash ~/ros2_ws/speaker.sh mute      # shut it up (testing in a room full of people)
#   bash ~/ros2_ws/speaker.sh unmute    # give it its voice back
#   bash ~/ros2_ws/speaker.sh status
#
# WHY THIS IS NOT JUST `amixer sset Speaker mute`.
#
# A muted robot gives NO sign of it. It still "speaks": text in the phone chat, text in the logs,
# the same timings, the same pauses for lines nobody hears. It just makes no sound. A guide robot
# narrating a whole demo in silence, on camera, in front of an investor, is a very quiet disaster --
# and the only warning you would get is an investor looking politely puzzled.
#
# So muting leaves a FLAG, and the flag is loud:
#
#   show_qr.sh   would otherwise unmute the speaker on every press (it was written to, for exactly
#                this reason). With the flag set it leaves it alone -- a deliberate mute survives
#                the button -- and says so on screen.
#   demo_check   FAILS. Not a warning: a failure. "NOT READY" until you give it its voice back.
#
# Unmuting removes the flag. There is no way to end up silent on Thursday without having been told.
FLAG="$HOME/.cache/robot_ds/speaker_muted"
CARD="Device"        # by NAME: plugging the mic array in renumbers the cards and 'card 1' then
                     # points at a microphone. This bit is not paranoia, it has happened.

case "${1:-status}" in
    mute)
        mkdir -p "$(dirname "$FLAG")"
        date +"muted %F %T -- run: bash ~/ros2_ws/speaker.sh unmute" > "$FLAG"
        amixer -c "$CARD" -q sset Speaker 0% mute 2>/dev/null
        echo "SPEAKER MUTED (on purpose)."
        echo "  The robot will still say everything -- you just won't hear it. Read it in the"
        echo "  phone's chat, or: journalctl -u robot-web -f | grep '\[robot\]'"
        echo
        echo "  demo_check.sh will FAIL until you run:  bash ~/ros2_ws/speaker.sh unmute"
        ;;
    unmute)
        rm -f "$FLAG"
        if amixer -c "$CARD" -q sset Speaker 100% unmute 2>/dev/null; then
            echo "SPEAKER ON, 100%. The robot has its voice back."
        else
            echo "could not set the speaker -- is the USB audio device plugged in?" >&2
            exit 1
        fi
        ;;
    status)
        if [ -f "$FLAG" ]; then
            echo "MUTED on purpose ($(cat "$FLAG"))"
            exit 1
        fi
        vol=$(amixer -c "$CARD" sget Speaker 2>/dev/null | grep -o '[0-9]*%' | head -1)
        state=$(amixer -c "$CARD" sget Speaker 2>/dev/null | grep -o '\[on\]\|\[off\]' | head -1)
        echo "speaker: ${vol:-?} ${state:-?}"
        [ "$state" = "[off]" ] && exit 1
        exit 0
        ;;
    vol|volume|level)
        # speaker.sh vol 60   -> set the speaker to 60% and unmute. A number, 0-100.
        # This is how the robot stops being all-or-nothing: testing next to colleagues you want it
        # quiet but audible, not silent. Setting a volume clears the deliberate-mute flag, because
        # asking for sound IS asking to be unmuted -- there is no "muted at 60%".
        n="${2:-}"
        case "$n" in
            ''|*[!0-9]*) echo "usage: speaker.sh vol <0-100>" >&2; exit 2 ;;
        esac
        [ "$n" -gt 100 ] && n=100
        rm -f "$FLAG"
        if [ "$n" -eq 0 ]; then
            # 0 is mute, and it must leave the flag so it survives the QR button (see mute above).
            mkdir -p "$(dirname "$FLAG")"
            date +"muted %F %T -- run: bash ~/ros2_ws/speaker.sh unmute" > "$FLAG"
            amixer -c "$CARD" -q sset Speaker 0% mute 2>/dev/null
            echo "speaker: 0% (muted)"
        elif amixer -c "$CARD" -q sset Speaker "${n}%" unmute 2>/dev/null; then
            echo "speaker: ${n}%"
        else
            echo "could not set the speaker -- is the USB audio device plugged in?" >&2
            exit 1
        fi
        ;;
    *)
        echo "usage: speaker.sh mute | unmute | vol <0-100> | status" >&2
        exit 2
        ;;
esac
