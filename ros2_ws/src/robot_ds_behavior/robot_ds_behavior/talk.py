#!/usr/bin/env python3
# talk.py - TALK to the robot from the terminal.
# You type in natural language (even indirectly) and the robot figures out where
# you want to go using Claude, then takes you there with Nav2. It's a LOOP: you
# give an order, the robot goes, and it waits for the next one -- like a chat.
#
#   "i'm hungry"              -> kitchen
#   "i'm going for my jacket" -> reception
#   "meeting with the team"   -> meeting
#   "back to my desk"         -> desks
#
# Same brain as brain.py (it imports route from there), but in an interactive
# loop. The robot STAYS where it arrives (it does not return to base): you give
# the next order from there. It also ANSWERS questions and, when it offers to
# take you somewhere, a "yes" on the next line makes it go.
#
# Usage (with Nav2 running):
#     python3 talk.py
import rclpy
try:  # installed package: ros2 run (the real robot, Humble)
    from robot_ds_behavior.guide import Guide, load_places, say   # guide node + places + TTS
    from robot_ds_behavior.brain import route, describe           # sentence -> action (Claude)
except ImportError:  # loose scripts: python3 talk.py (the sim on rosita)
    from guide import Guide, load_places, say
    from brain import route, describe

# words that end the conversation
QUIT = {"quit", "exit", "bye", "stop", "q", "done"}


def main():
    rclpy.init()
    node = Guide()
    print("==== Talk to the robot (type 'quit' to exit) ====")
    print("Places I know:", ", ".join(load_places()))
    print("Tell me things like: 'i'm hungry', 'what is the glass room?'...")
    pending = None                       # place we just offered (a 'yes' confirms it)
    try:
        while True:
            try:
                text = input("\nYou> ").strip()
            except EOFError:
                break
            if not text:
                continue
            if text.lower() in QUIT:
                print("See you!")
                break
            action, place, reply = route(text, pending=pending)
            if action == "say":
                pending = place          # remember an offer; forget if there's none
                say(reply)
                continue
            if action == "tour":
                pending = None
                print(f"[Tour -> {place}]")
                say(reply)               # intro
                for n in place:          # place is the ordered list of stops
                    node.go(n, arrive_say=describe(n))
                say("That's the end of the tour. Thanks for coming along!")
                continue
            if action != "go":
                pending = None
                say("Sorry, I didn't get that. Could you say it again?")
                continue
            pending = None
            print(f"[Claude understands -> {place}]")
            # `reply` is the arrival narration from the SAME route() call (no extra call)
            node.go(place, arrive_say=(reply or describe(place)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
