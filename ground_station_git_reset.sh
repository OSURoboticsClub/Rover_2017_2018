#!/bin/bash
test -d ~/Maps_Backup || mkdir -p ~/Maps_Backup 
cp -r software/ros_packages/ground_station/src/Resources/Maps/* ~/Maps_Backup
git reset --hard
git clean -fdx
git pull
test -d ~/software/ros_packages/ground_station/src/Resources/Maps || mkdir -p software/ros_packages/ground_station/src/Resources/Maps 
cp -r ~/Maps_Backup/* software/ros_packages/ground_station/src/Resources/Maps/
