#!/bin/bash
#
#
# make all the following files are in the current directory
# tmf8829.ko tmf8829-overlay-fpc-spi.dtbo tmf8829_application.hex
# this script run with su privilege.
set -e

if grep -Fxq "dtoverlay=i2c0,pins_28_29" /boot/firmware/config.txt
then
    echo "1. No need to modify /boot/firmware/config.txt"
else
    echo "1. Append dtoverlay=i2c0,pins_28_29 to /boot/firmware/config.txt"
    echo "dtoverlay=i2c0,pins_28_29" >> /boot/firmware/config.txt
fi

if grep -Fxq "dtparam=spi=on" /boot/firmware/config.txt
then
    echo "3. No need to modify /boot/firmware/config.txt"
else
    echo "3. Append dtparam=spi=on to /boot/firmware/config.txt"
    echo "dtparam=spi=on" >> /boot/firmware/config.txt
fi

if grep -Fxq "dtoverlay=tmf8829-overlay-fpc-spi" /boot/firmware/config.txt
then
    echo "3. No need to modify /boot/firmware/config.txt"
else
    echo "3. Append dtoverlay=tmf8829-overlay-fpc-spi to /boot/firmware/config.txt"
    echo "dtoverlay=tmf8829-overlay-fpc-spi" >> /boot/firmware/config.txt
fi

cp tmf8829-overlay-fpc-spi.dtbo /boot/overlays/
cp tmf8829_application.hex /lib/firmware/tmf8829_application.hex
cp tmf8829.ko /opt/USBSensorBridgeRuntime/modules/
cp ams-usb-sensorbridge.service /etc/systemd/system
cp tmf8829_zmq_server.service /etc/systemd/system/
