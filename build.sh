#!/bin/bash

echo "Hello"

sudo apt install build-essential make git -y

mkdir -p ~/.local/bin || true
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH=$PATH:$HOME/.local/bin
arduino-cli config init --overwrite
arduino-cli core update-index
arduino-cli core install rp2040:rp2040 &> /dev/null
arduino-cli lib install "TinyGPSPlus"
arduino-cli lib install "RTClib"
arduino-cli lib install "Time"
arduino-cli lib install "Etherkit Si5351"

# pwd
# find .

cpath=`pwd`

cd "ADX FIRMWARE/EXPERIMENTAL/PDX_V2.0" && arduino-cli compile --fqbn=rp2040:rp2040:rpipico -e . && cp build/rp2040.rp2040.rpipico/PDX_V2.0.ino.uf2 "$cpath"

exit 0
