#!/bin/bash

sleep 1

killall BLE_Student_ID_Scanner

tar xzvf Student_Project.tar.gz -C /tmp/

a=`cat /tmp/md5sum | awk "{print $1}"`

b=`md5sum BLE_Student_ID_Scanner*`

echo $a
echo $b

