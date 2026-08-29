#!/usr/bin/env python3
"""map_edit.py - anadir/quitar paredes en el mapa guardado por coordenadas.

Para paredes que el lidar NO detecto (cristales, muros bajos, huecos...) y que
quieres que Nav2 trate como obstaculo. Mismo truco que el keepout de cristales.
Hace COPIA DE SEGURIDAD automatica antes de cada cambio (para 'undo').

Uso (coordenadas en METROS, las mismas que los waypoints / la rejilla del mapa):
  python3 ~/ros2_ws/map_edit.py show                    # dibuja el mapa+rejilla -> /tmp/map_edit.png
  python3 ~/ros2_ws/map_edit.py wall X1 Y1 X2 Y2 [cm]   # pared entre 2 puntos (grosor 20cm por defecto)
  python3 ~/ros2_ws/map_edit.py free X1 Y1 X2 Y2 [cm]   # LIBERAR (borrar una pared mal puesta)
  python3 ~/ros2_ws/map_edit.py undo                    # deshacer el ultimo cambio

Tras editar: recarga el mapa en Navegacion (reinicia navigation + 2D Pose Estimate).
Coge las coordenadas mirando /tmp/map_edit.png (rojo=X, azul=Y) o marcando waypoints.
"""
import os
import sys
import math
import shutil
import yaml
from PIL import Image, ImageDraw, ImageFont

MAPD = os.path.expanduser("~/ros2_ws/src/slam/maps")
PGM = os.path.join(MAPD, "map_01.pgm")
YAMLF = os.path.join(MAPD, "map_01.yaml")
BK = os.path.expanduser("~/map_backups/map_01_before_last_edit.pgm")
OUT = "/tmp/map_edit.png"


def load():
    m = yaml.safe_load(open(YAMLF))
    res = float(m["resolution"])
    ox, oy = m["origin"][:2]
    return Image.open(PGM).convert("L"), res, float(ox), float(oy)


def w2p(x, y, res, ox, oy, H):
    return ((x - ox) / res, H - (y - oy) / res)


def render():
    img, res, ox, oy = load()
    W, H = img.size
    S = 1.6
    im = img.convert("RGB").resize((int(W * S), int(H * S)), Image.NEAREST)
    d = ImageDraw.Draw(im)
    try:
        f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 15)
    except Exception:
        f = None
    Iw, Ih = im.size
    gx = math.ceil(ox / 2) * 2
    while gx <= ox + W * res:
        px = (gx - ox) / res * S
        d.line([(px, 0), (px, Ih)], fill=(230, 60, 60), width=1)
        d.text((px + 2, 2), f"{gx:.0f}", fill=(200, 0, 0), font=f)
        gx += 2
    gy = math.ceil(oy / 2) * 2
    while gy <= oy + H * res:
        py = (H - (gy - oy) / res) * S
        d.line([(0, py), (Iw, py)], fill=(60, 90, 230), width=1)
        d.text((3, py + 1), f"{gy:.0f}", fill=(0, 0, 210), font=f)
        gy += 2
    im.save(OUT)
    print(f"Mapa -> {OUT}   (abrelo:  eog {OUT} )")


def edit(kind, x1, y1, x2, y2, cm):
    img, res, ox, oy = load()
    W, H = img.size
    os.makedirs(os.path.dirname(BK), exist_ok=True)
    shutil.copy(PGM, BK)                       # backup para 'undo'
    val = 0 if kind == "wall" else 254         # negro=obstaculo, blanco=libre
    th = max(1, int(round((cm / 100.0) / res)))
    a = w2p(x1, y1, res, ox, oy, H)
    b = w2p(x2, y2, res, ox, oy, H)
    d = ImageDraw.Draw(img)
    d.line([a, b], fill=val, width=th)
    for p in (a, b):                           # extremos redondeados (sin huecos)
        r = th / 2.0
        d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=val)
    img.save(PGM)
    label = "PARED anadida" if kind == "wall" else "LIBERADO"
    print(f"{label}: ({x1},{y1}) -> ({x2},{y2}), {cm:.0f} cm. Backup en {BK}")
    render()


def undo():
    if os.path.exists(BK):
        shutil.copy(BK, PGM)
        print("Deshecho: mapa restaurado desde antes del ultimo cambio.")
        render()
    else:
        print("No hay backup del ultimo cambio (nada que deshacer).")


def main():
    a = sys.argv[1:]
    if not a or a[0] == "show":
        render()
    elif a[0] == "undo":
        undo()
    elif a[0] in ("wall", "free") and len(a) >= 5:
        cm = float(a[5]) if len(a) > 5 else 20.0
        edit(a[0], float(a[1]), float(a[2]), float(a[3]), float(a[4]), cm)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
