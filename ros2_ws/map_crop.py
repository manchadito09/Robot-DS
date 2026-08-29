#!/usr/bin/env python3
# map_crop.py - REALLY cut the map down to the part of the floor we use.
#
# Painting an area black only hides it; this CROPS the .pgm so the unwanted half
# does not exist at all, and fixes `origin` in the .yaml so every world coordinate
# (and therefore every saved waypoint) stays exactly where it was.
#
#   python3 ~/ros2_ws/map_crop.py preview
#       -> writes /tmp/map_preview.png: the map with a 1 m grid, labelled every 5 m,
#          plus the saved waypoints. Open it and read off the cut you want.
#
#   python3 ~/ros2_ws/map_crop.py crop --xmin -2 --xmax 12 --ymin -6 --ymax 8
#       -> KEEPS that world rectangle, throws the rest away. Backs the map up first
#          and warns about waypoints that would fall outside.
#
# After cropping: relaunch Navigation (map_server loads the map at startup).
import os
import sys
import math
import time
import shutil
import argparse
import yaml
import cv2
import numpy as np

MAPS = os.path.expanduser("~/ros2_ws/src/slam/maps")
PGM = os.path.join(MAPS, "map_01.pgm")
YAML = os.path.join(MAPS, "map_01.yaml")
WPS = os.path.join(MAPS, "map_01.waypoints.yaml")
PREVIEW = "/tmp/map_preview.png"


def load_map():
    with open(YAML) as f:
        meta = yaml.safe_load(f) or {}
    img = cv2.imread(PGM, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit("Could not read %s" % PGM)
    res = float(meta.get("resolution", 0.05))
    ox, oy = (meta.get("origin") or [0, 0, 0])[:2]
    return img, res, float(ox), float(oy), meta


def load_waypoints():
    try:
        with open(WPS) as f:
            return (yaml.safe_load(f) or {}).get("waypoints") or {}
    except OSError:
        return {}


def w2p(wx, wy, res, ox, oy, h):
    """World metres -> pixel (col, row). Row 0 is the top (ROS origin is bottom-left)."""
    return (int(round((wx - ox) / res)), int(round(h - (wy - oy) / res)))


def cmd_preview():
    img, res, ox, oy, _ = load_map()
    h, w = img.shape[:2]
    s = 2                                        # upscale so the labels are readable
    vis = cv2.resize(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), (w * s, h * s),
                     interpolation=cv2.INTER_NEAREST)
    x_min, x_max = ox, ox + w * res
    y_min, y_max = oy, oy + h * res

    for x in range(math.ceil(x_min), math.floor(x_max) + 1):       # vertical lines
        px = w2p(x, 0, res, ox, oy, h)[0] * s
        big = (x % 5 == 0)
        cv2.line(vis, (px, 0), (px, h * s), (0, 150, 255) if big else (205, 205, 205), 2 if big else 1)
        if big:
            cv2.putText(vis, "x=%d" % x, (px + 4, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 150, 255), 2, cv2.LINE_AA)
    for y in range(math.ceil(y_min), math.floor(y_max) + 1):       # horizontal lines
        py = w2p(0, y, res, ox, oy, h)[1] * s
        big = (y % 5 == 0)
        cv2.line(vis, (0, py), (w * s, py), (0, 150, 255) if big else (205, 205, 205), 2 if big else 1)
        if big:
            cv2.putText(vis, "y=%d" % y, (6, py - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 150, 255), 2, cv2.LINE_AA)

    for name, p in load_waypoints().items():                        # saved points
        px, py = w2p(float(p["x"]), float(p["y"]), res, ox, oy, h)
        px, py = px * s, py * s
        cv2.circle(vis, (px, py), 7, (0, 200, 0), -1)
        cv2.putText(vis, name, (px + 9, py - 7), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 230, 0), 2, cv2.LINE_AA)

    cv2.imwrite(PREVIEW, vis)
    print("map is %d x %d px, %.2f m/px" % (w, h, res))
    print("world bounds:  x %.2f .. %.2f   y %.2f .. %.2f  (metres)" % (x_min, x_max, y_min, y_max))
    print("waypoints:", ", ".join(sorted(load_waypoints())) or "(none)")
    print("preview written to", PREVIEW)


def cmd_crop(xmin, xmax, ymin, ymax, yes=False):
    img, res, ox, oy, meta = load_map()
    h, w = img.shape[:2]
    x0, y1 = w2p(xmin, ymin, res, ox, oy, h)     # bottom-left  -> left col, bottom row
    x1, y0 = w2p(xmax, ymax, res, ox, oy, h)     # top-right    -> right col, top row
    x0, x1 = max(0, min(w, x0)), max(0, min(w, x1))
    y0, y1 = max(0, min(h, y0)), max(0, min(h, y1))
    if x1 - x0 < 2 or y1 - y0 < 2:
        sys.exit("That rectangle is empty or outside the map.")

    out = img[y0:y1, x0:x1]
    new_h, new_w = out.shape[:2]
    # origin = world coords of the new image's bottom-left corner
    new_ox = ox + x0 * res
    new_oy = oy + (h - y1) * res

    lost = [n for n, p in load_waypoints().items()
            if not (xmin <= float(p["x"]) <= xmax and ymin <= float(p["y"]) <= ymax)]

    print("keep world rect: x %.2f..%.2f  y %.2f..%.2f" % (xmin, xmax, ymin, ymax))
    print("new map: %d x %d px (was %d x %d)" % (new_w, new_h, w, h))
    print("new origin: [%.3f, %.3f, 0.0]  (was [%.3f, %.3f])" % (new_ox, new_oy, ox, oy))
    if lost:
        print("WARNING - these waypoints fall OUTSIDE and become unreachable:")
        for n in lost:
            print("   -", n)
    if not yes:
        if input("Write this? [y/N] ").strip().lower() != "y":
            sys.exit("Aborted, nothing changed.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    shutil.copy(PGM, PGM + "." + stamp + ".bak")
    shutil.copy(YAML, YAML + "." + stamp + ".bak")
    cv2.imwrite(PGM, out)
    meta["origin"] = [round(new_ox, 4), round(new_oy, 4), 0.0]
    with open(YAML, "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print("done. Backups: *.%s.bak" % stamp)
    print("Now RELAUNCH Navigation so map_server loads the new map.")


def cmd_erase(names, fill, yes=False):
    """Wipe a POLYGON (given by saved waypoints, in order) out of the map. Use this
    when the area to drop is rotated/diagonal -- a crop can only ever be an
    axis-aligned rectangle, so it would also swallow good ground.
      fill=unknown -> 205, the map shows it as never-scanned (needs allow_unknown:false)
      fill=wall    -> 0, a solid obstacle (blocked whatever the planner params say)"""
    img, res, ox, oy, _ = load_map()
    h, w = img.shape[:2]
    wps = load_waypoints()
    if len(names) < 3:
        sys.exit("Give at least 3 waypoints to make a polygon.")
    pts = []
    for n in names:
        if n not in wps:
            sys.exit('No waypoint named "%s". Saved: %s' % (n, ", ".join(sorted(wps))))
        pts.append(w2p(float(wps[n]["x"]), float(wps[n]["y"]), res, ox, oy, h))
    poly_i = np.array(pts, dtype=np.int32)
    poly_f = np.array(pts, dtype=np.float32)
    value = 205 if fill == "unknown" else 0

    lost = [n for n, p in wps.items() if n not in names and
            cv2.pointPolygonTest(poly_f, w2p(float(p["x"]), float(p["y"]), res, ox, oy, h), False) >= 0]

    before = int((img == value).sum())
    print("polygon: %s" % " -> ".join(names))
    print("fill: %s (%d)" % (fill, value))
    if lost:
        print("WARNING - these waypoints are INSIDE and become unreachable:")
        for n in lost:
            print("   -", n)
    else:
        print("no saved waypoints fall inside.")
    if not yes:
        if input("Wipe it? [y/N] ").strip().lower() != "y":
            sys.exit("Aborted, nothing changed.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    shutil.copy(PGM, PGM + "." + stamp + ".bak")
    cv2.fillPoly(img, [poly_i], value)
    cv2.imwrite(PGM, img)
    after = int((img == value).sum())
    print("done: %d cells wiped. Backup: %s" % (after - before, PGM + "." + stamp + ".bak"))
    if fill == "unknown":
        print("NOTE: set `allow_unknown: false` in nav2_params.yaml, or the planner may still route through it.")
    print("Now RELAUNCH Navigation so map_server loads the new map.")


def cmd_autocrop(margin, min_blob, radius, dry, yes=False):
    """Tidy the map and shrink the image to what is really there, so the map isn't
    lost in a sea of grey after erasing half the floor:
      1. drop tiny islands of free space (SLAM speckle) - they were stretching the
         bounding box out to the old, huge size,
      2. drop walls that no longer border any free space,
      3. crop to the bounding box of what's left (+ margin) and fix `origin`.
    World coordinates - and therefore every waypoint - do not move."""
    img, res, ox, oy, meta = load_map()
    h, w = img.shape[:2]
    work = img.copy()

    free = (work > 250).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(free, 8)
    speck = 0
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_blob:
            work[lab == i] = 205
            speck += int(stats[i, cv2.CC_STAT_AREA])

    walls = (work == 0).astype(np.uint8)
    free2 = (work > 250).astype(np.uint8)
    k = 2 * int(radius) + 1
    orphan = (walls == 1) & (cv2.dilate(free2, np.ones((k, k), np.uint8)) == 0)
    work[orphan] = 205

    ys, xs = np.where(work != 205)
    if len(xs) == 0:
        sys.exit("Nothing left after the tidy-up - check --min-blob.")
    x0, x1 = max(0, int(xs.min()) - margin), min(w, int(xs.max()) + 1 + margin)
    y0, y1 = max(0, int(ys.min()) - margin), min(h, int(ys.max()) + 1 + margin)
    new_w, new_h = x1 - x0, y1 - y0
    new_ox = ox + x0 * res
    new_oy = oy + (h - y1) * res

    print("removed %d speckle cells (free islands < %d px) and %d orphan wall cells"
          % (speck, min_blob, int(orphan.sum())))
    print("new map: %d x %d px  (%.1f x %.1f m)   was %d x %d px (%.1f x %.1f m)"
          % (new_w, new_h, new_w * res, new_h * res, w, h, w * res, h * res))
    print("new origin: [%.3f, %.3f, 0.0]  (was [%.3f, %.3f])" % (new_ox, new_oy, ox, oy))
    lost = [n for n, p in load_waypoints().items()
            if not (new_ox <= float(p["x"]) <= new_ox + new_w * res
                    and new_oy <= float(p["y"]) <= new_oy + new_h * res)]
    print("WARNING - waypoints falling OUTSIDE: " + ", ".join(lost) if lost
          else "all %d waypoints stay inside." % len(load_waypoints()))
    if dry:
        print("(dry run, nothing written)")
        return
    if not yes:
        if input("Write this? [y/N] ").strip().lower() != "y":
            sys.exit("Aborted, nothing changed.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    shutil.copy(PGM, PGM + "." + stamp + ".bak")
    shutil.copy(YAML, YAML + "." + stamp + ".bak")
    cv2.imwrite(PGM, work[y0:y1, x0:x1])
    meta["origin"] = [round(new_ox, 4), round(new_oy, 4), 0.0]
    with open(YAML, "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print("done. Backups: *.%s.bak" % stamp)
    print("Now RELAUNCH Navigation so map_server loads the new map.")


def cmd_line(names, thickness, fill, yes=False):
    """Draw a wall between saved waypoints -- a virtual barrier that seals an opening
    left by an erased area, so the planner keeps the robot away from it.
      fill=wall (default) -> 0, a solid obstacle · fill=unknown -> 205"""
    img, res, ox, oy, _ = load_map()
    h, w = img.shape[:2]
    wps = load_waypoints()
    if len(names) < 2:
        sys.exit("Give at least 2 waypoints to draw a line.")
    pts = []
    for n in names:
        if n not in wps:
            sys.exit('No waypoint named "%s". Saved: %s' % (n, ", ".join(sorted(wps))))
        pts.append(w2p(float(wps[n]["x"]), float(wps[n]["y"]), res, ox, oy, h))
    value = 0 if fill == "wall" else 205

    print("line: %s" % " -> ".join(names))
    print("thickness: %d px (%.0f cm)   fill: %s (%d)" % (thickness, thickness * res * 100, fill, value))
    if not yes:
        if input("Draw it? [y/N] ").strip().lower() != "y":
            sys.exit("Aborted, nothing changed.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    shutil.copy(PGM, PGM + "." + stamp + ".bak")
    before = int((img == value).sum())
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, value, int(thickness))
    cv2.imwrite(PGM, img)
    print("done: %d cells drawn. Backup: *.%s.bak" % (int((img == value).sum()) - before, stamp))
    print("Now RELAUNCH Navigation so map_server loads the new map.")


def cmd_clean(radius, dry, yes=False):
    """Drop wall pixels that no longer border any free space -- the stray lines left
    floating in an erased (unknown) area. Walls that still bound the floor we keep
    touch free space, so they survive."""
    img, _res, _ox, _oy, _m = load_map()
    walls = (img == 0).astype(np.uint8)
    free = (img > 250).astype(np.uint8)
    k = 2 * int(radius) + 1
    near_free = cv2.dilate(free, np.ones((k, k), np.uint8))     # free space, grown by `radius`
    orphan = (walls == 1) & (near_free == 0)                    # wall with no free space nearby
    n = int(orphan.sum())
    print("walls: %d   orphan walls (no free space within %d px): %d"
          % (int(walls.sum()), radius, n))
    if dry:
        print("(dry run, nothing written)")
        return
    if n == 0:
        print("nothing to clean.")
        return
    if not yes:
        if input("Wipe those %d stray wall cells? [y/N] " % n).strip().lower() != "y":
            sys.exit("Aborted, nothing changed.")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    shutil.copy(PGM, PGM + "." + stamp + ".bak")
    img[orphan] = 205                                           # back to 'never mapped'
    cv2.imwrite(PGM, img)
    print("done: %d stray wall cells wiped. Backup: *.%s.bak" % (n, stamp))
    print("Now RELAUNCH Navigation so map_server loads the new map.")


def corners_from_waypoints(a, b):
    """Two saved waypoints = the OPPOSITE CORNERS of the rectangle to keep. Easier
    than typing metres: drop the two points on the map, then name them here."""
    wps = load_waypoints()
    for n in (a, b):
        if n not in wps:
            sys.exit('No waypoint named "%s". Saved: %s' % (n, ", ".join(sorted(wps)) or "(none)"))
    ax, ay = float(wps[a]["x"]), float(wps[a]["y"])
    bx, by = float(wps[b]["x"]), float(wps[b]["y"])
    return min(ax, bx), max(ax, bx), min(ay, by), max(ay, by)


def main():
    ap = argparse.ArgumentParser(description="Crop the SLAM map to the part of the floor we use.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("preview", help="write /tmp/map_preview.png with a metre grid + waypoints")
    c = sub.add_parser("crop", help="keep a world rectangle, throw the rest away")
    c.add_argument("--corners", nargs=2, metavar=("POINT_A", "POINT_B"),
                   help="two saved waypoints used as opposite corners of the area to KEEP")
    for a in ("xmin", "xmax", "ymin", "ymax"):
        c.add_argument("--" + a, type=float)
    c.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")

    e = sub.add_parser("erase", help="wipe a polygon (rotated/diagonal areas) out of the map")
    e.add_argument("--points", nargs="+", required=True, metavar="WAYPOINT",
                   help="3+ saved waypoints, in order around the area to wipe")
    e.add_argument("--fill", choices=("unknown", "wall"), default="unknown",
                   help="unknown = looks never-scanned (default) · wall = solid obstacle")
    e.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")

    cl = sub.add_parser("clean", help="drop stray walls left floating in erased areas")
    cl.add_argument("--radius", type=int, default=3,
                    help="a wall is kept if free space is within this many pixels (default 3 = 15 cm)")
    cl.add_argument("--dry", action="store_true", help="only count, write nothing")
    cl.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")

    ac = sub.add_parser("autocrop", help="tidy speckle + shrink the image to the mapped area")
    ac.add_argument("--margin", type=int, default=10, help="pixels of padding to keep (default 10 = 50 cm)")
    ac.add_argument("--min-blob", type=int, default=500, dest="min_blob",
                    help="free islands smaller than this (px) are SLAM speckle (default 500 = 1.25 m2)")
    ac.add_argument("--radius", type=int, default=3, help="wall is kept if free space is within N px")
    ac.add_argument("--dry", action="store_true", help="only show the new size, write nothing")
    ac.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")

    ln = sub.add_parser("line", help="draw a virtual wall between saved waypoints")
    ln.add_argument("--points", nargs="+", required=True, metavar="WAYPOINT",
                    help="2+ saved waypoints; a wall is drawn from one to the next")
    ln.add_argument("--thickness", type=int, default=3, help="pixels thick (default 3 = 15 cm)")
    ln.add_argument("--fill", choices=("wall", "unknown"), default="wall")
    ln.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")

    a = ap.parse_args()
    if a.cmd == "preview":
        cmd_preview()
        return
    if a.cmd == "line":
        cmd_line(a.points, a.thickness, a.fill, a.yes)
        return
    if a.cmd == "autocrop":
        cmd_autocrop(a.margin, a.min_blob, a.radius, a.dry, a.yes)
        return
    if a.cmd == "erase":
        cmd_erase(a.points, a.fill, a.yes)
        return
    if a.cmd == "clean":
        cmd_clean(a.radius, a.dry, a.yes)
        return
    if a.corners:
        xmin, xmax, ymin, ymax = corners_from_waypoints(*a.corners)
        print("corners %s / %s -> keep x %.2f..%.2f  y %.2f..%.2f"
              % (a.corners[0], a.corners[1], xmin, xmax, ymin, ymax))
    elif None not in (a.xmin, a.xmax, a.ymin, a.ymax):
        xmin, xmax, ymin, ymax = a.xmin, a.xmax, a.ymin, a.ymax
    else:
        sys.exit("Give either --corners A B (two waypoints) or all of --xmin --xmax --ymin --ymax.")
    cmd_crop(xmin, xmax, ymin, ymax, a.yes)


if __name__ == "__main__":
    main()
