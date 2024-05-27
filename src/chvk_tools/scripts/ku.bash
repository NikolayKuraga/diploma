#!/bin/sh


# Script to set system up. A.


# For Xubuntu.
gsettings set org.gnome.desktop.lockdown disable-lock-screen true
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.screensaver ubuntu-lock-on-suspend false
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
gsettings set org.gnome.desktop.session idle-delay 0


# Python dependencies for every-src-for-catkin.
sudo apt-get install python3-scipy python3-numpy


# libopencv 4.2* from Ubuntu instead of 4.5 from Jetson repos.
sudo apt-get install libopencv-dev=4.2*
sudo apt-get autoremove --purge libopencv # single 4.5 package
sudo apt-mark hold libopencv-dev
