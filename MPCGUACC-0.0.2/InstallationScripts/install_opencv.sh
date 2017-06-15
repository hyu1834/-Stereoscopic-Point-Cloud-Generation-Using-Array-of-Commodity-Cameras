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
sudo apt-get -y install build-essential cmake cmake-curses-gui pkg-config 
sudo apt-get -y install libpng12-0 libpng12-dev libpng++-dev libpng3 libpnglite-dev 
sudo apt-get -y install zlib1g-dbg zlib1g zlib1g-dev 
sudo apt-get -y install pngtools 
sudo apt-get -y install libtiff5-dev libtiff5 libtiffxx0c2 libtiff-tools libeigen3-dev
sudo apt-get -y install libjpeg8 libjpeg8-dev libjpeg8-dbg libjpeg-progs 
sudo apt-get -y install libjasper-dev libpng12-dev
sudo apt-get -y install libavcodec-dev libavformat-dev 
sudo apt-get -y install libgstreamer0.10-0-dbg libgstreamer0.10-0 libgstreamer0.10-dev 
sudo apt-get -y install libunicap2 libunicap2-dev 
sudo apt-get -y install swig libv4l-0 libv4l-dev 
sudo apt-get -y install python-numpy libpython2.7 python2.7-dev python-dev 
sudo apt-get -y install libgtk2.0-dev
sudo apt-get -y install libswscale-dev libv4l-dev
sudo apt-get -y install libatlas-base-dev gfortran
sudo apt-get -y install libopencv-dev python-opencv


# download openCV2.4.10
#wget -O opencv-2.4.11.zip http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.11/opencv-2.4.11.zip/download
#unzip opencv-2.4.11.zip
#cd opencv-2.4.11
# make opencv
#make
#sudo make install
#sudo ldconfig

# link library
#cd ~/.virtualenvs/cv/lib/python2.7/site-packages/
#ln -s /usr/local/lib/python2.7/site-packages/cv2.so cv2.so
#ln -s /usr/local/lib/python2.7/site-packages/cv.py cv.py



