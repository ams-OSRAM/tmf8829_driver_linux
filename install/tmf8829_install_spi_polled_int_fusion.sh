#!/bin/bash
#
#
# make all the following files are in the current directory
# tmf8829.ko tmf8829-overlay-fpc-spi-polled.dtbo tmf8829_application.hex
# this script run with su privilege.
set -e

if grep -Fxq "dtoverlay=i2c0,pins_28_29" /boot/config.txt
then
    echo "1. Uncomment dtoverlay=i2c0,pins_28_29"
    sed -i 's/dtoverlay=i2c0,pins_28_29/#dtoverlay=i2c0,pins_28_29/' /boot/config.txt
else
    echo "1. No need to modify /boot/config.txt"
fi

if grep -Fxq "dtparam=spi=on" /boot/config.txt
then
    echo "2. No need to modify /boot/config.txt"
else
    echo "2. Append dtparam=spi=on to /boot/config.txt"
    echo "dtparam=spi=on" >> /boot/config.txt
fi

if grep -Fxq "dtoverlay=tmf8829-overlay-fpc-spi-polled-fusion" /boot/config.txt
then
    echo "3. No need to modify /boot/config.txt"
else
    echo "3. Append dtoverlay=tmf8829-overlay-fpc-spi-polled to /boot/config.txt"
    echo "dtoverlay=tmf8829-overlay-fpc-spi-polled" >> /boot/config.txt
fi

cp tmf8829-overlay-fpc-spi-polled-fusion.dtbo /boot/overlays/
cp tmf8829_application.hex /lib/firmware/tmf8829_application.hex
cp rp2_fusion_board/tmf8829.ko /lib/modules/$(uname -r)/kernel/drivers/
depmod -a