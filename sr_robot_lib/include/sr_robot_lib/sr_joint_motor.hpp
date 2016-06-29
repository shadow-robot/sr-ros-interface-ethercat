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

#include <string>
#include <vector>

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

struct JointToMuscle
{
  int muscle_driver_id[2];
  int muscle_id[2];
};

class SrActuatorWrapper
{
public:
  SrActuatorWrapper()
          : actuator(NULL),
            actuator_ok(false),
            bad_data(false)
  {
  }

  // actuator
  ros_ethercat_model::Actuator *actuator;

  /**
   * this boolean is set to true as long as we receive the
   * data from the actuator.
   */
  bool actuator_ok;
  /**
   * this boolean is set to true if the data coming from the actuator
   * through the CAN bus are messed up.
   */
  bool bad_data;
};

class MotorWrapper :
        public SrActuatorWrapper
{
public:
  MotorWrapper()
          : motor_id(0),
            msg_motor_id(0)
  {
  }

  // the position of the motor in the motor array
  // coming from the hardware
  int motor_id;

  // the position of the motor in the message array
  int msg_motor_id;

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

class MuscleDriver
{
public:
  explicit MuscleDriver(int id = 0)
          : muscle_driver_id(id),
            can_msgs_received_(0),
            can_msgs_transmitted_(0),
            pic_firmware_git_revision_(0),
            server_firmware_git_revision_(0),
            firmware_modified_(0),
            serial_number(0),
            assembly_date_year(0),
            assembly_date_month(0),
            assembly_date_day(0),
            can_err_tx(0),
            can_err_rx(0),
            driver_ok(false),
            bad_data(false)
  {
  }

  int muscle_driver_id;
  unsigned int pic_firmware_git_revision_;
  unsigned int server_firmware_git_revision_;
  bool firmware_modified_;
  unsigned int serial_number;
  unsigned int assembly_date_year;
  unsigned int assembly_date_month;
  unsigned int assembly_date_day;

  unsigned int can_err_tx;
  unsigned int can_err_rx;
  uint64_t can_msgs_transmitted_;
  uint64_t can_msgs_received_;

  bool driver_ok;
  bool bad_data;

  /**
   * A service used to reset the HW
   * muscle controller
   */
  ros::ServiceServer reset_driver_service;
};

class MuscleWrapper :
        public SrActuatorWrapper
{
public:
  MuscleWrapper()
          : muscle_id(),
            muscle_driver_id()
  {
  }

  /// id of the muscle drivers that control the muscles for this joint. These muscles can be driven
  // by different muscle drivers.
  int muscle_driver_id[2];
  /// id of the muscles for this joint (the id indicates the order from 0 to 9 of the muscle in its muscle driver)
  int muscle_id[2];
};

struct Joint
{
  std::string joint_name;

  // the indexes of the joints in the joint array
  // coming from the hardware which are used to
  // compute the joint data.
  JointToSensor joint_to_sensor;

  // used to filter the position and the velocity
  sr_math_utils::filters::LowPassFilter pos_filter;
  // used to filter the effort
  sr_math_utils::filters::LowPassFilter effort_filter;

  bool has_actuator;
  boost::shared_ptr<SrActuatorWrapper> actuator_wrapper;
};

typedef threadsafe::Map<boost::shared_ptr<shadow_robot::JointCalibration> > CalibrationMap;
}  // namespace shadow_joints

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */

#endif /* SR_JOINT_MOTOR_HPP_ */
