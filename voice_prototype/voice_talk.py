#!/usr/bin/env python3
# voice_talk.py - TALK TO THE ROBOT BY VOICE (sim front-end).
#
# Ties the laptop voice pipeline to the robot's brain on rosita:
#   mic -> STT (faster-whisper) -> send the text to rosita's brain (Claude picks
#   a POI, Nav2 drives) -> the narration streams back -> TTS speaks it out loud.
#
# It's the voice face of talk.py: same brain (brain.py on rosita), but you SPEAK
# instead of typing and the robot ANSWERS out loud.
#
# On the REAL robot all of this runs on the robot itself (no SSH): the 6-mic
# array and speaker are on board and brain.py/guide.py run locally. Here, voice
# lives on the laptop and Nav2 on rosita, so we bridge over SSH (ssh rosita ->
# ~/voice_nav.sh -> brain.py --stay).
#
# Run (Windows, no venv activation needed):
#     .venv\Scripts\python.exe voice_talk.py
import shlex
import subprocess
from record import record_until_enter
from stt import transcribe
from tts import speak

ROSITA = "rosita"   # ssh host running Nav2 + the robot brain


def navigate_and_speak(text):
    """Send the recognized text to rosita's brain; speak the narration it streams."""
    remote = "bash ~/voice_nav.sh " + shlex.quote(text)
    proc = subprocess.Popen(["ssh", ROSITA, remote],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    spoke = False
    for line in proc.stdout:
        line = line.strip()
        if line.startswith("[robot]"):                 # guide.py narration
            said = line[len("[robot]"):].strip()
            print("  robot:", said)
            speak(said, "en")
            spoke = True
        elif line.startswith("Claude picked:"):
            print("  ->", line)
    proc.wait()
    if not spoke:
        speak("Sorry, I'm not sure where to take you.", "en")


def main():
    print("==== Voice guide (Ctrl+C to quit) ====")
    print("Press ENTER, speak (e.g. \"I'm hungry\"), ENTER -- the robot listens and drives.")
    while True:
        try:
            record_until_enter("audio.wav")
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break
        text = (transcribe("audio.wav")[0] or "").strip()
        print("You said:", repr(text))
        if not text:
            print("(heard nothing -- try again)")
            continue
        navigate_and_speak(text)


if __name__ == "__main__":
    main()
