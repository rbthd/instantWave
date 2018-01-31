#!/bin/bash

## Purge system from unused programs
sudo apt-get --purge remove wolfram-engine bluej greenfoot nodered nuscratch scratch sonic-pi libreoffice* claws-mail claws-mail-i18n minecraft-pi python-pygame

sudo apt-get autoremove --purge

sudo apt-get clean

sudo apt-get update && sudo apt-get upgrade -y ## VERY long instance
sudo apt full-upgrade -y

## Install needed libraries and packages

sudo apt-get install -y geany
sudo apt-get install -y i2c-tools
sudo apt-get install -y netatalk

## Enable sharing between raspberry and Mac OS

sudo update-rc.d avahi-daemon defaults


sudo echo "<?xml version= »1.0″ standalone=’no’?><!–*-nxml-*–>" >> /etc/avahi/services/afpd.service
sudo echo "<!DOCTYPE service-group SYSTEM « avahi-service.dtd »>" >> /etc/avahi/services/afpd.service
sudo echo "<service-group>" >> /etc/avahi/services/afpd.service
sudo echo "<name replace-wildcards= »yes »>%h</name>" >> /etc/avahi/services/afpd.service
sudo echo "<service>" >> /etc/avahi/services/afpd.service
sudo echo "<type>_afpovertcp._tcp</type>" >> /etc/avahi/services/afpd.service
sudo echo "<port>548</port>" >> /etc/avahi/services/afpd.service
sudo echo "</service>" >> /etc/avahi/services/afpd.service
sudo echo "</service-group>" >> /etc/avahi/services/afpd.service

sudo /etc/init.d/avahi-daemon restart

##
