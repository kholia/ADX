#!/bin/bash

echo "Hello! :-)"

# Install system packages
sudo apt-get install build-essential make git patch -y &>/dev/null
sudo apt-get install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib -y &>/dev/null

# Install toolchain
mkdir -p ~/.local/bin || true
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH=$PATH:$HOME/.local/bin
arduino-cli config init --overwrite
arduino-cli core update-index
arduino-cli core install rp2040:rp2040 &> /dev/null
arduino-cli lib install "TinyGPSPlus" &> /dev/null
arduino-cli lib install "RTClib" &> /dev/null
arduino-cli lib install "Time" &> /dev/null
arduino-cli lib install "Etherkit Si5351" &> /dev/null

# Debugging steps
# pwd
# find .

# Patch toolchain (hacky)
cpath=`pwd`
ver="2.5.2"
cd "ADX FIRMWARE/EXPERIMENTAL/patches"
cp Serial* ~/.arduino15/packages/rp2040/hardware/rp2040/$ver/cores/rp2040/
cp cdc* ~/.arduino15/packages/rp2040/hardware/rp2040/$ver/pico-sdk/lib/tinyusb/src/class/cdc/
cd ~/.arduino15/packages/rp2040/hardware/rp2040/$ver/tools/libpico
echo "Building patched libpico.a, this may take a while..."
sh ./make-libpico.sh &>/dev/null

# Build firmware
cd "$cpath"
cd "ADX FIRMWARE/EXPERIMENTAL/PDX_V2.0"
sed -i 's/\/\/#define\ HAVE_USB_PATCHES/#define\ HAVE_USB_PATCHES/ g' PDX_V2.0.ino
cat PDX_V2.0.ino | grep HAVE_USB_PATCHES
arduino-cli compile --fqbn=rp2040:rp2040:rpipico -e . && cp build/rp2040.rp2040.rpipico/PDX_V2.0.ino.uf2 "$cpath"

exit 0
