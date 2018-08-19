#!/bin/bash

path="$PWD/wpa_supplicant.conf"
backup_path="$PWD/wpa_supplicant.conf_back"
destination_path="/etc/wpa_supplicant/wpa_supplicant.conf"
sudo cp $PWD/interfaces /etc/network/
cp $path $backup_path
echo "network={" >> $backup_path
echo "\tssid=\"$1\"" >> $backup_path
echo "\tpsk=\"$2\"" >> $backup_path
echo "\tpriority=2" >> $backup_path
echo "}" >> $backup_path

#wpa_passphrase $1 $2 >> $backup_path
#sudo cp $backup_path $PWD/test.txt
sudo cp $backup_path $destination_path
sudo ifdown wlan0
sudo ifup wlan0
sleep 3

