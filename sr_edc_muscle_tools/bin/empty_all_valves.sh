#!/bin/bash

DURATION=5000

# We need valve controllers for this
rosrun sr_edc_muscle_tools start_valve_controllers.sh

# Command valves to open
echo Opening valves...
for muscle in ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2; do
    rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, -4]
cmd_duration_ms: [$DURATION,$DURATION]" &
done

