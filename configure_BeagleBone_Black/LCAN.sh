#!/bin/bash
sudo cp BB-DCAN1-00A0.dtbo /lib/firmware
echo BB-DCAN1 > /sys/devices/bone_capemgr.*/slots
sudo modprobe can
sudo modprobe can-dev
sudo modprobe can-raw
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up
echo "Interface CAN0 on, bitrate 1000000, use candump to read messages"