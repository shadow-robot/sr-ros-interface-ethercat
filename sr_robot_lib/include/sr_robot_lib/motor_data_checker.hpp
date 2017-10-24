/**
 * @file   motor_data_checker.hpp
 * @author toni <toni@shadowrobot.com>
 * @date   25 Oct 2011
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
 * @brief This is a class to check that all expected initialization data have been received from each motor.
 *
 *
 */

#ifndef MOTOR_DATA_CHECKER_HPP_
#define MOTOR_DATA_CHECKER_HPP_

#include <ros/ros.h>
#include <vector>
#include <boost/array.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "sr_robot_lib/sr_joint_motor.hpp"
#include "sr_robot_lib/generic_updater.hpp"

extern "C"
{
#include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
#include <sr_external_dependencies/external/0240_palm_edc_IMU/0240_palm_edc_IMU_ethercat_protocol.h>
}

namespace generic_updater
{
class MessageFromMotorChecker
{
public:
  explicit MessageFromMotorChecker(int id)
          : motor_id_(id), received_(false)
  {
  }

  virtual ~MessageFromMotorChecker()
  {
  }

  int motor_id_;

  virtual void set_received();

  bool get_received();

protected:
  bool received_;
};

class SlowMessageFromMotorChecker :
        public MessageFromMotorChecker
{
public:
  explicit SlowMessageFromMotorChecker(int id);

  boost::array<bool, MOTOR_SLOW_DATA_LAST + 1> slow_data_received;

  virtual void set_received(FROM_MOTOR_SLOW_DATA_TYPE slow_data_type);
};

class MessageChecker
{
public:
  explicit MessageChecker(FROM_MOTOR_DATA_TYPE msg_type)
          : msg_type(msg_type)
  {
  }

  FROM_MOTOR_DATA_TYPE msg_type;
  std::vector<MessageFromMotorChecker *> msg_from_motor_checkers;

  int find(int motor_id);
};

/**
 * MotorDataChecker checks if all expected messages from the motors
 * have been received
 */
class MotorDataChecker
{
public:
  MotorDataChecker(std::vector<shadow_joints::Joint> joints_vector,
                   std::vector<UpdateConfig> initialization_configs_vector);

  ~MotorDataChecker();

  /**
   * Checks the message as received. Checking if we received the specified motor_data_type
   * or the motor_slow_data_type.
   * Checks a certain message coming from a certain joint (motor)
   * Joints without a motor are not expected to provide any information
   *
   * @param joint_tmp joint iterator containing the data of the joint
   * @param motor_data_type the type of the received data
   * @param motor_slow_data_type the type of the received sub-data (used if the motor_data_type is MOTOR_DATA_SLOW_MISC)
   * @return true if all expected messages have already been received
   */
  bool check_message(std::vector<shadow_joints::Joint>::iterator joint_tmp,
                     FROM_MOTOR_DATA_TYPE motor_data_type, int16u motor_slow_data_type);

  /**
   * Initializes the Motor Data Checker to the not received state for each message
   * Should be used when reinitializing
   *
   * @param joints_vector the vector with the joints (motors) from which information is coming
   * @param initialization_configs_vector vector containing the initialization commands whose answers need to be checked
   */
  void init(std::vector<shadow_joints::Joint> joints_vector,
            std::vector<UpdateConfig> initialization_configs_vector);

protected:
  static const double timeout;
  ros::NodeHandle nh_tilde;
  ros::Timer check_timeout_timer;
  operation_mode::device_update_state::DeviceUpdateState update_state;
  ros::Duration init_max_duration;

  void timer_callback(const ros::TimerEvent &event);

  bool is_everything_checked();

  int find(FROM_MOTOR_DATA_TYPE motor_data_type);

  std::vector<MessageChecker> msg_checkers_;
};
}  // namespace generic_updater

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */

#endif /* MOTOR_DATA_CHECKER_HPP_ */
