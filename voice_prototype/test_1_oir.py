"""
PRUEBA de la Pieza 1: ¿oye y entiende lo que digo?
Graba lo que dices y te lo escribe en pantalla. Sin Claude todavía.
"""

from record import record_until_enter
from stt import transcribe

wav = record_until_enter()
print("Transcribiendo...")
texto, idioma = transcribe(wav)
print()
print(f"  Idioma detectado: {idioma}")
print(f"  Has dicho: \"{texto}\"")
