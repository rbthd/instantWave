#!/bin/bash

## Pull github code
mkdir instantWave


https://github.com/DavidEGrayson/minimu9-ahrs.git
https://github.com/DavidEGrayson/ahrs-visualizer.git

## Installing required libraries
sudo apt-get install libi2c-dev libeigen3-dev libboost-program-options-dev

#Configuring i2C

sudo nano /etc/modules >> i2c-bcm2708
sudo nano /etc/modules >> i2c-dev

sudo nano /boot/config.txt >> dtparam=i2c1=on
sudo nano /boot/config.txt >> dtparam=i2c_arm=on
