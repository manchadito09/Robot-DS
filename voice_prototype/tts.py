"""
Etapa 4 — TTS (Text To Speech) 🔊
Convierte el texto de la respuesta en voz por el altavoz.

Elige el motor automáticamente según la máquina:
- En Linux (el robot): usa `espeak` (local, offline, gratis).
- En Windows (el portátil): usa pyttsx3 con las voces SAPI5.

Así el mismo archivo vale para probar en el portátil y para correr en el robot.
"""

import sys
import shutil
import subprocess

# Mini-programa para Windows: dice argv[1] con la voz pyttsx3 argv[2], en un
# proceso aparte (reutilizar SAPI en el mismo proceso deja mudas las siguientes).
_WORKER = r"""
import sys, pyttsx3
text = sys.argv[1]
want = sys.argv[2] if len(sys.argv) > 2 else ""
e = pyttsx3.init()
e.setProperty("rate", 175)
if want:
    for v in e.getProperty("voices"):
        if want in (v.name + " " + v.id).lower():
            e.setProperty("voice", v.id)
            break
e.say(text)
e.runAndWait()
"""


def _want_voice(lang):
    """Nombre de voz pyttsx3 (Windows) según el idioma."""
    lang = (lang or "").lower()
    if lang.startswith("es"):
        return "spanish"
    if lang.startswith("en"):
        return "english"
    return ""


def _espeak_voice(lang):
    """Código de voz espeak (Linux) según el idioma."""
    lang = (lang or "").lower()
    if lang.startswith("es"):
        return "es"
    return "en-us"


def speak(text, lang=None):
    """Dice 'text' en voz alta. 'lang' = código de idioma ('en', 'es', ...)."""
    if not text:
        return
    espeak = shutil.which("espeak") or shutil.which("espeak-ng")
    if espeak:
        # Linux (robot): voz por espeak. -s = velocidad (más bajo = más lento).
        subprocess.run([espeak, "-v", _espeak_voice(lang), "-s", "150", text])
    else:
        # Windows (portátil): voz por pyttsx3 en un proceso aparte.
        subprocess.run([sys.executable, "-c", _WORKER, text, _want_voice(lang)])


if __name__ == "__main__":
    # Prueba suelta: dice una frase en cada idioma con el motor que toque.
    speak("Hi! I'm Wall-E, your floor four guide. Follow me to the kitchen.", "en")
    speak("¡Hola! Soy Wall-E, tu guía de la planta cuatro. Sígueme a la cocina.", "es")
