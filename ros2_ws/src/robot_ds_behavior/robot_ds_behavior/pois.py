#!/usr/bin/env python3
# pois.py - the robot's LIST OF PLACES (POIs = Points Of Interest).
#
# Kept separate from the code on purpose: for the real robot you only edit the
# coordinates HERE, without touching guide.py or brain.py.
#
# Format:  "name": (x, y)   in metres, in the scene's WORLD frame.
# These match the floor markers (m_rec / m_meet / m_kit / m_desk / m_exit) that
# step1b.py drops in front of each real area (reception desk, meeting table,
# kitchen counter, desks, exit gap).
POIS = {
    "reception": (-3.5, 2.3),
    "meeting": (3.0, 1.8),
    "kitchen": (3.5, -2.5),
    "desks": (-3.0, -1.8),
    "exit": (0.0, -3.3),
    "center": (0.0, 0.0),
}

# World (scene) -> map (Nav2 / SLAM) calibration: a planar RIGID transform
# (rotation MAP_YAW + translation MAP_OFFSET). The POIs above are in the scene's
# WORLD frame; Nav2 navigates in the SLAM "map" frame, which is rotated AND
# shifted vs world, so guide.py converts each goal:
#     map = R(MAP_YAW) * world + MAP_OFFSET
#
# Measured 2026-06-18 for the CURRENT sim map (robot_worldpose.py + tf2_echo).
# RE-MEASURE if SLAM is restarted -- the map origin changes every session.
# On the REAL robot this is the identity (POIs get tagged straight in the floor
# map): set MAP_YAW = 0.0 and MAP_OFFSET = (0.0, 0.0).
MAP_YAW = 0.16704
MAP_OFFSET = (-0.5963, 3.2486)
