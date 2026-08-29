"""
Stage 2 — STT (Speech To Text) 📝
Turns the .wav into text using faster-whisper (local, free, multilingual).
The first time it downloads the model (a few hundred MB); after that it's offline.
"""

from faster_whisper import WhisperModel

# Load the model once and reuse it (this is the slow part).
_model = None


def get_model(size="base.en"):
    """Load the Whisper model. 'base' = fast on CPU; 'small' = more accurate but slower."""
    global _model
    if _model is None:
        print(f"Loading Whisper model '{size}' (downloads on first run)...")
        # device='cpu' + compute_type='int8' = fast and light without a GPU.
        _model = WhisperModel(size, device="cpu", compute_type="int8")
    return _model


def transcribe(path="audio.wav", language=None):
    """Transcribe a .wav. Returns (text, detected_language).
    language=None auto-detects (can pick a wrong language and hallucinate on
    noisy/short audio); pass e.g. "en" to lock it and stop the random-language
    gibberish."""
    model = get_model()
    # vad_filter=True drops silence -> avoids Whisper hallucinations when the
    # audio is empty/quiet, and speeds it up. beam_size=1 = faster.
    segments, info = model.transcribe(path, vad_filter=True, beam_size=1,
                                      language=language)
    text = "".join(seg.text for seg in segments).strip()
    return text, info.language


if __name__ == "__main__":
    # Standalone test: transcribe the audio.wav left by record.py.
    text, lang = transcribe()
    print(f"[{lang}] {text}")
