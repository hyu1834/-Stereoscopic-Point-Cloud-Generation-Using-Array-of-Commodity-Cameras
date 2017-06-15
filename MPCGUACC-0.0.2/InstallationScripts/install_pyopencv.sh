#! /bin/bash
# Author: Hiu Hong Yu
# Orginatzion: UC Davis
# Description:
# NOTE: THIS SCRIPT FOR Raspberry Pi ONLY
# Supported Raspberry Pi
# - Raspberry Pi B+
# - Raspberry Pi 2 B
# - Raspberry Pi 3 B
# 

# make sure the system is up to date
sudo apt-get update
sudo apt-get upgrade
sudo rpi-update

# install dependencies
sudo apt-get install -y python-numpy libpython2.7 python2.7-dev python-dev 
sudo apt-get install -y libopencv-dev python-opencv

