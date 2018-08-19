#!/bin/bash

sudo killall BLE_Student_ID_Scanner

sleep 1

sudo cp /home/pi/BLE_Student_Project/Student_Project.tar.gz /home/pi/

sudo rm -rf /home/pi/BLE_Student_Project/

sudo tar xzvf /home/pi/Student_Project.tar.gz -C /home/pi/

cd /home/pi/BLE_Student_Project/

sudo make

sleep 1

sudo ./BLE_Student_ID_Scanner
