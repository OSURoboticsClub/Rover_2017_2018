#!/bin/bash
cp -r software/ros_packages/ground_station/src/Maps ~/Maps_Backup
git reset --hard
git clean -fdx
git pull
cp -r ~/Maps_Backup software/ros_packages/ground_station/src/Maps
