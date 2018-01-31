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

sudo nano /etc/avahi/services/afpd.service >> <?xml version= »1.0″ standalone=’no’?><!–*-nxml-*–>
sudo nano /etc/avahi/services/afpd.service >> <!DOCTYPE service-group SYSTEM « avahi-service.dtd »>
sudo nano /etc/avahi/services/afpd.service >> <service-group>
sudo nano /etc/avahi/services/afpd.service >> <name replace-wildcards= »yes »>%h</name>
sudo nano /etc/avahi/services/afpd.service >> <service>
sudo nano /etc/avahi/services/afpd.service >> <type>_afpovertcp._tcp</type>
sudo nano /etc/avahi/services/afpd.service >> <port>548</port>
sudo nano /etc/avahi/services/afpd.service >> </service>
sudo nano /etc/avahi/services/afpd.service >> </service-group>

sudo /etc/init.d/avahi-daemon restart

##
