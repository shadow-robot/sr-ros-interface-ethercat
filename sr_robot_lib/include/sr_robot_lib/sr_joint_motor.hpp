/**
 * @file   sr_joint_motor.hpp
 * @author toni <toni@shadowrobot.com>
 * @date   26 Oct 2011
 *
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @brief Contains the definitions of Motor and Joint
 *
 *
 */

#ifndef SR_JOINT_MOTOR_HPP_
#define SR_JOINT_MOTOR_HPP_

#include <sr_hardware_interface/sr_actuator.hpp>

#include <sr_utilities/sr_math_utils.hpp>
#include <sr_utilities/calibration.hpp>
#include <sr_utilities/thread_safe_map.hpp>


namespace shadow_joints
{
  struct PartialJointToSensor
  {
    int sensor_id;
    double coeff;
  };

  struct JointToSensor
  {
    std::vector<std::string> sensor_names;
    std::vector<PartialJointToSensor> joint_to_sensor_vector;
    bool calibrate_after_combining_sensors;
  };

  class Motor
  {
  public:
    Motor()
        : motor_id(0), msg_motor_id(0), actuator(NULL), motor_ok(false), bad_data(false)
    {
    }
    ;

    ~Motor()
    {
    }
    ;

    //the position of the motor in the motor array
    // coming from the hardware
    int motor_id;

    //the position of the motor in the message array
    int msg_motor_id;

    //actuator
    sr_actuator::SrActuator* actuator;

    /**
     * this boolean is set to true as long as we receive the
     * data from the motor.
     */
    bool motor_ok;
    /**
     * this boolean is set to true if the data coming from the motor
     * through the CAN bus are messed up.
     */
    bool bad_data;

    /**
     * A service used to set the force PID settings on the
     * motor.
     */
    ros::ServiceServer force_pid_service;

    /**
     * A service used to reset the
     * motors.
     */
    ros::ServiceServer reset_motor_service;
  };

  struct Joint
  {
    std::string joint_name;

    //the indexes of the joints in the joint array
    // coming from the hardware which are used to
    // compute the joint data.
    JointToSensor joint_to_sensor;

    //used to filter the position and the velocity
    sr_math_utils::filters::LowPassFilter pos_filter;
    //used to filter the effort
    sr_math_utils::filters::LowPassFilter effort_filter;

    bool has_motor;
    boost::shared_ptr<Motor> motor;
  };

  typedef threadsafe::Map<boost::shared_ptr<shadow_robot::JointCalibration> > CalibrationMap;
}

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */

#endif /* SR_JOINT_MOTOR_HPP_ */
