"""
EL BUCLE COMPLETO 🎙️🧠🔊
Junta las 4 etapas: grabar -> transcribir -> Claude decide -> hablar.
Pulsa ENTER para hablarle al robot; Ctrl+C para salir.

Esto es el prototipo de voz del robot-guía, en el portátil. En el robot real,
el `destino` que imprime se convierte en un objetivo de navegación (Nav2) y la
respuesta se dice por el altavoz mientras lleva al visitante.
"""

from record import record_until_enter
from stt import transcribe
from brain import decide
from tts import speak


def main():
    print("=" * 50)
    print(" Concierge de voz — robot-guía planta 4")
    print(" ENTER para hablar · Ctrl+C para salir")
    print("=" * 50)
    while True:
        wav = record_until_enter()
        print("Transcribiendo...")
        text, lang = transcribe(wav)
        if not text:
            print("(no te he oído, repite)")
            continue
        print(f'[{lang}] Tú: "{text}"')

        dest, reply = decide(text, lang)
        if dest:
            print(f"🧭 Destino: {dest}")
        print(f"🤖 {reply}")
        speak(reply, lang)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n¡Hasta luego!")
