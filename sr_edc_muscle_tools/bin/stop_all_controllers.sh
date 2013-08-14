#!/bin/bash

for name in $(rosrun pr2_controller_manager pr2_controller_manager list | grep running | cut -d ' ' -f 1); do
    rosrun pr2_controller_manager pr2_controller_manager stop $name
done
