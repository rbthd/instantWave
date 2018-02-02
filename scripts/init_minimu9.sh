#!/bin/bash

## Pull github code
#sudo git clone https://github.com/rbthd/instantWave

## Installing required libraries
sudo apt-get install libi2c-dev libeigen3-dev libboost-program-options-dev

#Configuring i2C

sudo su
echo "i2c-bcm2708" >> /etc/modules
echo "i2c-dev" >> /etc/modules >>

echo "dtparam=i2c1=on" >> /boot/config.txt
echo "dtparam=i2c_arm=on" >> /boot/config.txt
exit


