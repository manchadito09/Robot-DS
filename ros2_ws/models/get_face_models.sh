#!/bin/bash
# get_face_models.sh - fetch the two face models "Add me" and face_greet.py need.
#
#   bash ~/ros2_ws/models/get_face_models.sh
#
# They are NOT in git: 37 MB of binary that never changes does not belong in a repo you clone
# over the robot's wifi. They come from OpenCV's own model zoo, and OpenCV 4.10 (already on this
# robot) knows how to run them with no extra library -- no dlib, no face_recognition, no ARM
# compile that eats an afternoon.
#
#   YuNet  ~230 KB  finds faces in a frame
#   SFace   ~37 MB  turns a face into the 128 numbers we compare
set -e
cd "$(dirname "$0")"
BASE="https://github.com/opencv/opencv_zoo/raw/main/models"

get () {   # name, url
    if [ -s "$1" ]; then
        echo "already here: $1"
        return
    fi
    echo "downloading $1 ..."
    curl -sSL -o "$1" "$2"
    # A repo that switches to git-lfs would hand us a 130-byte pointer file instead of a model,
    # and OpenCV's error for that is not obvious. Catch it here, where it is.
    if head -c 60 "$1" | grep -q "git-lfs"; then
        rm -f "$1"
        echo "ERROR: got an LFS pointer, not the model. Download $1 by hand from opencv_zoo." >&2
        exit 1
    fi
}

get face_detection_yunet_2023mar.onnx    "$BASE/face_detection_yunet/face_detection_yunet_2023mar.onnx"
get face_recognition_sface_2021dec.onnx  "$BASE/face_recognition_sface/face_recognition_sface_2021dec.onnx"

echo
ls -lh ./*.onnx | awk '{print "  "$9"  "$5}'
echo "Done. Restart robot-web so 'Add me' picks them up:  sudo systemctl restart robot-web"
