"""
Diagnóstico de micrófono 🎚️
Lista los dispositivos de entrada y muestra un medidor de nivel EN VIVO, para
encontrar qué micro capta bien y con qué índice.

Uso:
    python mic_check.py            # mide el micro por defecto de Windows
    python mic_check.py 3          # mide el dispositivo de índice 3

Cuando encuentres el índice que sube de nivel al hablar, ponlo en record.py
(variable MIC_DEVICE).
"""

import sys
import numpy as np
import sounddevice as sd

print("=== Dispositivos de audio (los de entrada tienen 'in') ===")
print(sd.query_devices())

dev = int(sys.argv[1]) if len(sys.argv) > 1 else None
info = sd.query_devices(dev, "input")
print(f"\nMidiendo entrada: {info['name']}")
print("Habla durante 6 segundos y mira la barra:\n")


def cb(indata, frames, t, status):
    peak = float(np.max(np.abs(indata)))
    bar = "#" * int(peak * 50)
    print(f"\r  {peak:4.2f} [{bar:<50}]", end="", flush=True)


with sd.InputStream(device=dev, channels=1, samplerate=16000, callback=cb):
    sd.sleep(6000)

print("\n\nFin. Si la barra subía al hablar (>0.20), ese micro vale.")
print("Si NO subía con ninguno, sube el volumen del micro en Ajustes de Windows.")
