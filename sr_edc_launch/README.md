# sr_edc_launch

This package contain launchfiles for the Shadow Robot EDC (EtherCAT Dual CAN) hand.

## Contents

- `sr_edc.launch` Launches a single hand

- `sr_edc_bimanual.launch` Launches 2 hands (as part of the same bimanual robot)


## Use

### Direct use

For a single hand, with default parameters:

```
roslaunch sr_edc_launch sr_edc.launch
```

For a bimanual system you will need to specify at least the serial numbers for the right and left hand:

```
roslaunch sr_edc_launch sr_edc_bimanual.launch rh_serial:=1234 lh_serial:=1235
```

### Use from another launchfile

E.g. for a bimanual system:

```
<launch>
  <include file="$(find sr_edc_launch)/sr_edc_bimanual.launch" >
    <arg name="rh_serial" value="1234" />
    <arg name="lh_serial" value="1235" />
    <arg name="eth_port" value="eth0_eth1" />
    <arg name="robot_description" value="$(find sr_description)/robots/bimanual_shadowhand_motor.urdf.xacro" />
  </include>
</launch>
```

E.g. for a single hand system:

```
<launch>
  <include file="$(find sr_edc_launch)/sr_edc.launch" >
    <arg name="eth_port" value="eth1" />
    <arg name="robot_description" value="$(find sr_description)/robots/shadowhand_motor_biotac.urdf.xacro" />
  </include>
</launch>
```

### Available arguments for sr_edc_bimanual.launch

- `eth_port`: The ethernet port/s that will be used to search for etherCAT devices (shadow hands or other devices like RoNeX). More than one port can be provided in this argument, using underscore as a separator.

```
roslaunch sr_edc_launch sr_edc_bimanual.launch rh_serial:=1234 lh_serial:=1235 eth_port:=eth0_eth1
```

- `debug`: Set to true for debugging
- `calibration_controllers`: Set to 0 if we don't want to run calibration controllers (e.g. for the muscle hand)
- `robot_description`: Xacro file containing the robot description we want to load
- `pwm_control`: The control mode PWM (true) or torque (false) 
- `rh_serial`: The ethercat serial number for the right hand
- `rh_id`: The id for the right hand. It needs to be the same (but without trailing underscore) as the prefix used in the hand model.
- `lh_serial`: The ethercat serial number for the left hand
- `lh_id`: The id for the left hand. It needs to be the same (but without trailing underscore) as the prefix used in the hand model.

### Available arguments for sr_edc.launch

- `eth_port`: The ethernet port/s that will be used to search for etherCAT devices (shadow hands or other devices like RoNeX). More than one port can be provided in this argument, using underscore as a separator.
- `debug`: Set to true for debugging
- `calibration_controllers`: Set to 0 if we don't want to run calibration controllers (e.g. for the muscle hand)
- `robot_description`: Xacro file containing the robot description we want to load
- `pwm_control`: The control mode PWM (true) or torque (false) 
- `config_dir`: Defines the sub-directory used in
       $(find sr_ethercat_hand_config)/mappings/
       and $(find sr_ethercat_hand_config)/calibrations/
       and $(find sr_ethercat_hand_config)/controls/host/
       to know where to find the parameters for a certain hand.
       In case we only have one hand this parameter will normally be "", meaning that the desired params are in the root of those directories
       If it's not "" then it must be followed by a "/" e.g. "hand_2/"
