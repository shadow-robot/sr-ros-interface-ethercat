^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_robot_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2015-04-07)
------------------
* Skip reset gains for arm controllers
* Skip reset_gains if the controller is a trajectory controller
* Fix diagnostics prefix
* Add muscle hand joint prefix
* Fix ns for controller resetting
* Use joint_prefix in order to get the right actuator from the hardware_interface.
* Set node handle for the tactile classes. Prefix the diagnostics.
* Set node handle from ouside the robot_lib. Prefix the diagnostics.
* Add ns_prefix and joint_prefix to hand devices.
* UBI0.cpp, sr_motor_robot_lib.cpp and sr_muscle_robot_lib now match upstream
  Removed sr_bringup as calibration file is now in ros_ethercat
* Increased UBI0 timeout to 10 seconds
* Now mid prox data are still extracted in UBI0 update, but do not require that the from_sensor_data_type is correct (as it is in fact incorrect when there is no distal tactile sensor connected)
* Added a 5s timeout to the tactile initialization. In case no tactiles are detected, we assume that we have UBI0 tactiles. This is a temporary solution that allow us to extract the middle, proximal and palm sensors even if no tactiles are plugged in

1.3.1 (2014-07-18)
------------------

1.3.0 (2014-02-14)
------------------
* first hydro release

