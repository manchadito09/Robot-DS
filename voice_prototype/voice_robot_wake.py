#!/usr/bin/env python3
# voice_robot_wake.py - HANDS-FREE voice guide, 100% on the robot.
#
#   say the WAKE WORD ("hey jarvis") -> beep -> speak your request -> the robot
#   transcribes it (faster-whisper), the brain picks a saved waypoint, Nav2
#   drives and it narrates out loud (Piper voice). Then it listens again.
#
# No keyboard: the 6-mic array listens continuously for the wake word
# (openWakeWord, local). The press-ENTER version is still voice_robot.py.
#
# Needs Nav2 launched and the robot localized (2D Pose Estimate in RViz).
# Run (from this folder):
#   .venv/bin/python voice_robot_wake.py
import os
import subprocess

# Silence onnxruntime's one-off "GPU device discovery failed" line (import-time
# only; we never touch stderr during recording/transcription).
_saved = os.dup(2)
_null = os.open(os.devnull, os.O_WRONLY)
os.dup2(_null, 2)
try:
    import onnxruntime
    onnxruntime.set_default_logger_severity(3)
except Exception:
    pass
finally:
    os.dup2(_saved, 2)
    os.close(_saved)
    os.close(_null)

import numpy as np
import sounddevice as sd
import soundfile as sf
from stt import transcribe, get_model
from openwakeword.model import Model

MIC_DEVICE = 0                                   # 6-mic array (index 0)
VOICE_NAV = os.path.expanduser("~/voice_nav.sh")
# By card NAME, never a number: plugging the 6-mic array in shifts the USB speaker from
# card 1 to card 0, and a hard-coded plughw:1,0 then plays into a microphone.
SPEAKER_DEV = os.environ.get("ROBOT_SPEAKER", "plughw:CARD=Device,DEV=0")
STT_LANG = "en"                                  # understand English only

WAKEWORD = os.environ.get("ROBOT_WAKEWORD", "hey_jarvis")  # or alexa / hey_mycroft / hey_rhasspy
WAKE_THRESHOLD = float(os.environ.get("ROBOT_WAKE_THRESHOLD", "0.5"))

RATE = 16000
CHUNK = 1280            # 80 ms @ 16 kHz (openWakeWord frame)
SPEECH_LEVEL = 900      # int16 peak above this = someone is talking
SILENCE_S = 1.0         # stop the command after this much trailing silence
MAX_CMD_S = 6.0         # ... or after this long, whichever comes first
BEEP_WAV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "beep.wav")


def ensure_beep():
    if os.path.exists(BEEP_WAV):
        return
    t = np.linspace(0, 0.12, int(RATE * 0.12), False)
    tone = (0.3 * np.sin(2 * np.pi * 880 * t) * 32767).astype(np.int16)
    sf.write(BEEP_WAV, tone, RATE, subtype="PCM_16")


def beep():
    subprocess.run(["aplay", "-q", "-D", SPEAKER_DEV, BEEP_WAV],
                   stderr=subprocess.DEVNULL)


def drive(text):
    """Send the text to the brain; the robot picks a goal, drives and speaks."""
    proc = subprocess.Popen(["bash", VOICE_NAV, text],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in proc.stdout:
        print("  " + line.rstrip())
    proc.wait()


def wake_and_capture(oww):
    """Block until the wake word, beep, then record the command until silence.
    Returns the command audio as an int16 numpy array (may be empty)."""
    with sd.InputStream(samplerate=RATE, channels=1, device=MIC_DEVICE,
                        dtype="int16", blocksize=CHUNK) as stream:
        oww.reset()
        # 1) wait for the wake word
        while True:
            data, _ = stream.read(CHUNK)
            mono = data[:, 0].copy()
            if oww.predict(mono).get(WAKEWORD, 0.0) >= WAKE_THRESHOLD:
                break
        beep()
        # drop the buffered beep echo so it isn't counted as speech
        stream.read(int(0.2 * RATE))
        # 2) record the command until trailing silence (or the max length)
        frames, silent, spoke, elapsed = [], 0.0, False, 0.0
        while elapsed < MAX_CMD_S:
            data, _ = stream.read(CHUNK)
            frames.append(data.copy())
            elapsed += CHUNK / RATE
            if int(np.max(np.abs(data))) > SPEECH_LEVEL:
                spoke, silent = True, 0.0
            elif spoke:
                silent += CHUNK / RATE
                if silent >= SILENCE_S:
                    break
    if not frames:
        return np.zeros(0, dtype=np.int16)
    return np.concatenate(frames)[:, 0]


def main():
    ensure_beep()
    print("Loading voice model...", flush=True)
    get_model()
    print(f"Loading wake word '{WAKEWORD}'...", flush=True)
    oww = Model(wakeword_models=[WAKEWORD], inference_framework="onnx")
    print(f'Ready. Say "{WAKEWORD.replace("_", " ")}" then your request '
          f'(Ctrl+C to quit)\n')
    while True:
        try:
            audio = wake_and_capture(oww)
        except KeyboardInterrupt:
            print("\nBye!")
            break
        sf.write("audio.wav", audio, RATE, subtype="PCM_16")
        text = (transcribe("audio.wav", language=STT_LANG)[0] or "").strip()
        print("You said:", repr(text))
        if not text:
            continue
        drive(text)


if __name__ == "__main__":
    main()
