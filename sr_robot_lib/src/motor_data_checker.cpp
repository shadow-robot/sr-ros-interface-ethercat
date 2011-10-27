/**
 * @file   motor_data_checker.cpp
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

#include "sr_robot_lib/motor_data_checker.hpp"

namespace generic_updater
{
  const double MotorDataChecker::timeout = 1.0;

  MotorDataChecker::MotorDataChecker(boost::ptr_vector<shadow_joints::Joint> joints_vector,
                                     std::vector<UpdateConfig> initialization_configs_vector)
      : nh_tilde("~"), update_state(operation_mode::device_update_state::INITIALIZATION), init_max_duration(timeout)
  {
    init(joints_vector, initialization_configs_vector);
  }

  void MotorDataChecker::init(boost::ptr_vector<shadow_joints::Joint> joints_vector,
                              std::vector<UpdateConfig> initialization_configs_vector)
  {
    //Create a one-shot timer
    check_timeout_timer = nh_tilde.createTimer(init_max_duration,
                                               boost::bind(&MotorDataChecker::timer_callback, this, _1), true);
    update_state = operation_mode::device_update_state::INITIALIZATION;
    msg_checkers_.clear();

    std::vector<UpdateConfig>::iterator msg_it;

    for (msg_it = initialization_configs_vector.begin(); msg_it < initialization_configs_vector.end(); msg_it++)
    {
      MessageChecker tmp_msg_checker( static_cast<FROM_MOTOR_DATA_TYPE>(msg_it->what_to_update) );
      boost::ptr_vector<shadow_joints::Joint>::iterator joint;
      for (joint = joints_vector.begin(); joint < joints_vector.end(); joint++)
      {
        if (joint->has_motor)
        {
          if (msg_it->what_to_update == MOTOR_DATA_SLOW_MISC)
          {
            SlowMessageFromMotorChecker tmp_msg_from_motor_checker(joint->motor->motor_id);
            tmp_msg_checker.msg_from_motor_checkers.push_back(tmp_msg_from_motor_checker);
          }
          else
          {
            MessageFromMotorChecker tmp_msg_from_motor_checker(joint->motor->motor_id);
            tmp_msg_checker.msg_from_motor_checkers.push_back(tmp_msg_from_motor_checker);
          }
        }
      }
      msg_checkers_.push_back(tmp_msg_checker);
    }
  }

  bool MotorDataChecker::check_message(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp,
                                       FROM_MOTOR_DATA_TYPE motor_data_type, int16u motor_slow_data_type)
  {
    std::vector<MessageChecker>::iterator it;

    it = find(motor_data_type);
    if (it != msg_checkers_.end())
    {
      std::vector<MessageFromMotorChecker>::iterator it2;
      it2 = it->find(joint_tmp->motor->motor_id);
      if (it2 != it->msg_from_motor_checkers.end())
      {
        if (motor_data_type == MOTOR_DATA_SLOW_MISC)
        {
          //we assume that the type of it2 is SlowMessageFromMotorChecker
          static_cast<SlowMessageFromMotorChecker*>( &(*it2) )->set_received( static_cast<FROM_MOTOR_SLOW_DATA_TYPE>(motor_slow_data_type) );
        }
        else
        {
          //we assume that the type of it2 is MessageFromMotorChecker
          it2->set_received();
        }
      }
    }

    return ((update_state == operation_mode::device_update_state::OPERATION) && is_everything_checked());
  }

  bool MotorDataChecker::is_everything_checked()
  {
    std::vector<MessageChecker>::iterator it;

    for (it = msg_checkers_.begin(); it < msg_checkers_.end(); it++)
    {
      std::vector<MessageFromMotorChecker>::iterator it2;

      for (it2 = it->msg_from_motor_checkers.begin(); it2 < it->msg_from_motor_checkers.end(); it2++)
      {
        if (!it2->get_received())
          return false;
      }
    }
    return true;
  }

  std::vector<MessageChecker>::iterator MotorDataChecker::find(FROM_MOTOR_DATA_TYPE motor_data_type)
  {
    std::vector<MessageChecker>::iterator it;

    for (it = msg_checkers_.begin(); it < msg_checkers_.end(); it++)
    {
      if (it->msg_type == motor_data_type)
        return it;
    }
    return msg_checkers_.end();
  }

  void MotorDataChecker::timer_callback(const ros::TimerEvent& event)
  {
    if( update_state == operation_mode::device_update_state::INITIALIZATION )
    {
      update_state = operation_mode::device_update_state::OPERATION;
      ROS_WARN_STREAM("Motor Initialization Timeout!!!!");
    }
  }

  std::vector<MessageFromMotorChecker>::iterator MessageChecker::find(int motor_id)
  {
    std::vector<MessageFromMotorChecker>::iterator it;

    for (it = msg_from_motor_checkers.begin(); it < msg_from_motor_checkers.end(); it++)
    {
      if (it->motor_id_ == motor_id)
        return it;
    }
    return msg_from_motor_checkers.end();
  }

  SlowMessageFromMotorChecker::SlowMessageFromMotorChecker(int id)
      : MessageFromMotorChecker(id)
  {
    for (int i = 0; i <= MOTOR_SLOW_DATA_LAST; i++)
    {
      slow_data_received.at(i) = false;
    }
  }

  void SlowMessageFromMotorChecker::set_received(FROM_MOTOR_SLOW_DATA_TYPE slow_data_type)
  {
    if (received_ == false)
    {
      //Check the slow data type as received
      if ( slow_data_type > MOTOR_SLOW_DATA_LAST )
      {
        ROS_ERROR_STREAM("Received bad slow_data_type: " << slow_data_type << " > " << slow_data_received.size() );
        return;
      }
      slow_data_received.at(slow_data_type) = true;

      //look if every type is received, then change FROM_MOTOR_SLOW_DATA_TYPE general received state accordingly
      bool checked = true;
      for (int i = MOTOR_SLOW_DATA_SVN_REVISION; i <= MOTOR_SLOW_DATA_LAST; i++)
      {
        checked &= slow_data_received.at(i);
        if (!checked)
        {
          ROS_INFO_STREAM(" still waiting for: " << i);
          break;
        }
      }
      if (checked)
        received_ = true;
    }
  }

  void MessageFromMotorChecker::set_received()
  {
    received_ = true;
  }

  bool MessageFromMotorChecker::get_received()
  {
    return received_;
  }

}

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
