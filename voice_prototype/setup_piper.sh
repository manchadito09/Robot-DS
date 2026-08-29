#!/usr/bin/env bash
# setup_piper.sh - download the Piper TTS binary + the robot's voice model.
# Piper is a standalone binary (not pip). This grabs the right build for your
# OS/arch and the male voice the robot uses (en_US-ryan-medium).
#
# Usage:  ./setup_piper.sh          # default voice: en_US-ryan-medium
#         ./setup_piper.sh en_US-lessac-medium   # a different voice
set -e
cd "$(dirname "$0")"
mkdir -p piper && cd piper

REL="2023.11.14-2"
OS=$(uname -s)
ARCH=$(uname -m)
case "$OS/$ARCH" in
  Linux/x86_64)   PKG="piper_linux_x86_64.tar.gz" ;;
  Linux/aarch64)  PKG="piper_linux_aarch64.tar.gz" ;;
  Linux/armv7l)   PKG="piper_linux_armv7l.tar.gz" ;;
  Darwin/x86_64)  PKG="piper_macos_x64.tar.gz" ;;
  Darwin/arm64)   PKG="piper_macos_aarch64.tar.gz" ;;
  *) echo "Unhandled $OS/$ARCH. Grab a build manually: https://github.com/rhasspy/piper/releases"; exit 1 ;;
esac

echo "==> Downloading Piper binary ($OS/$ARCH)..."
curl -fsSL -o piper.tar.gz "https://github.com/rhasspy/piper/releases/download/$REL/$PKG"
tar xzf piper.tar.gz && rm -f piper.tar.gz

VOICE="${1:-en_US-ryan-medium}"
# path on HuggingFace: en/<lang>/<name>/<quality>/<voice>.onnx  (e.g. en/en_US/ryan/medium/...)
lang=$(echo "$VOICE" | cut -d- -f1)                 # en_US
speaker=$(echo "$VOICE" | cut -d- -f2)              # ryan
quality=$(echo "$VOICE" | cut -d- -f3)              # medium
BASE="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/$lang/$speaker/$quality"
mkdir -p models
echo "==> Downloading voice $VOICE..."
curl -fsSL -o "models/$VOICE.onnx"      "$BASE/$VOICE.onnx"
curl -fsSL -o "models/$VOICE.onnx.json" "$BASE/$VOICE.onnx.json"

echo "==> Done."
echo "    Test it:  echo 'Hello, I am your guide robot.' | ./piper/piper --model models/$VOICE.onnx --output_file /tmp/t.wav && aplay /tmp/t.wav"
