#!/bin/bash

# We want only the valve controllers running, so stop all running
for name in $(rosrun pr2_controller_manager pr2_controller_manager list | grep running | cut -d ' ' -f 1); do
    rosrun pr2_controller_manager pr2_controller_manager stop $name
done
# Start valve controllers
for muscle in ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2; do
    rosrun pr2_controller_manager pr2_controller_manager spawn "sh_${muscle}_muscle_valve_controller"
done
