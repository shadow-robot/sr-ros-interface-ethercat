#!/bin/bash

DURATION=20000

for muscle in ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2; do
rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [4, -4]
cmd_duration_ms: [$DURATION,$DURATION]"
echo $muscle >> foo.out
sleep 20
rostopic echo -n1 /sh_"$muscle"_muscle_valve_controller/state/muscle_pressure_0 >> foo.out

rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, 4]
cmd_duration_ms: [$DURATION,$DURATION]"
sleep 20
rostopic echo -n1 /sh_"$muscle"_muscle_valve_controller/state/muscle_pressure_1 >> foo.out

rostopic pub -1 /sh_"$muscle"_muscle_valve_controller/command sr_robot_msgs/JointMuscleValveControllerCommand "cmd_valve_muscle: [-4, -4]
cmd_duration_ms: [$DURATION,$DURATION]"

done
exit
