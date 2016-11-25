#!/bin/bash
# Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com
# The BeagleBone Black doesn't have a battery for backing-up the system time.
# So when the BeagleBone Black wakes up using this script scheduled with the cron daemon
# it can synchronize its time.

TIME_SERVER_IP=192.168.7.1
sudo /etc/init.d/chrony stop
echo "Waiting for time synchronization with time server ${TIME_SERVER_IP}"
sudo ntpdate ${TIME_SERVER_IP}
sudo /etc/init.d/chrony start
