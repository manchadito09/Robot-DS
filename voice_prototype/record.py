"""
Stage 1 - RECORD
Records what you say through the mic and saves it to a .wav.
"Press-ENTER-to-talk" mode: ENTER starts, ENTER stops.
Records at 16 kHz mono, which is exactly what Whisper expects.
"""

import threading
import numpy as np
import sounddevice as sd
import soundfile as sf

SAMPLE_RATE = 16000  # 16 kHz: the format Whisper wants
CHANNELS = 1         # mono
MIC_DEVICE = None    # index of the mic to use (None = the Windows default).
                     # If the level comes out low, find the good one with
                     # mic_check.py and set its index here, e.g. MIC_DEVICE = 3


def record_until_enter(out_path="audio.wav", device=None):
    """Record from the mic until the user presses ENTER. Returns the .wav path."""
    if device is None:
        device = MIC_DEVICE
    input("Press ENTER to start recording...")
    print("🔴 Recording... speak now. Press ENTER to stop.")

    # A separate thread waits for the second ENTER while the mic keeps capturing.
    stop = threading.Event()
    threading.Thread(target=lambda: (input(), stop.set()), daemon=True).start()

    # Write the audio to the file as it arrives from the mic.
    with sf.SoundFile(out_path, mode="w", samplerate=SAMPLE_RATE,
                      channels=CHANNELS, subtype="PCM_16") as f:
        with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, device=device,
                            callback=lambda indata, frames, t, s: f.write(indata.copy())):
            while not stop.is_set():
                sd.sleep(100)

    # Only speak up when the mic barely caught anything (Whisper would
    # hallucinate on near-silence); otherwise stay quiet to keep the log clean.
    try:
        data, _ = sf.read(out_path)
        peak = float(np.max(np.abs(data))) if len(data) else 0.0
        bar = "#" * int(peak * 40)
        warn = "  ⚠️ VERY LOW (check the mic)" if peak < 0.05 else ""
        print(f"🎚️  peak level: {peak:4.2f} [{bar:<40}]{warn}")
    except Exception:
        pass

    return out_path


if __name__ == "__main__":
    # Standalone test: record and leave audio.wav ready.
    record_until_enter()
