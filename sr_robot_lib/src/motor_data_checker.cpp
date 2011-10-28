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
  const double MotorDataChecker::timeout = 15.0;

  MotorDataChecker::MotorDataChecker(boost::ptr_vector<shadow_joints::Joint> joints_vector,
                                     std::vector<UpdateConfig> initialization_configs_vector)
      : nh_tilde("~"), update_state(operation_mode::device_update_state::INITIALIZATION), init_max_duration(timeout)
  {
    init(joints_vector, initialization_configs_vector);
  }

  MotorDataChecker::~MotorDataChecker()
  {
    int i,j;
    for(i=0; i<msg_checkers_.size();i++)
    {
      for(j=0; j<msg_checkers_.at(i).msg_from_motor_checkers.size();j++)
      {
        delete msg_checkers_.at(i).msg_from_motor_checkers.at(j);
      }
    }
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
            tmp_msg_checker.msg_from_motor_checkers.push_back(new SlowMessageFromMotorChecker(joint->motor->motor_id));
          }
          else
          {
            tmp_msg_checker.msg_from_motor_checkers.push_back(new MessageFromMotorChecker(joint->motor->motor_id));
          }
        }
      }
      msg_checkers_.push_back(tmp_msg_checker);
    }

    //TODO remove
    ROS_WARN_STREAM("Msg checkers size: " << msg_checkers_.size() );
    int i,j;
    for(i=0; i<msg_checkers_.size();i++)
    {
      ROS_WARN_STREAM("  Msg checker type: " << msg_checkers_.at(i).msg_type );
      ROS_WARN_STREAM("  Motor checker size: " << msg_checkers_.at(i).msg_from_motor_checkers.size() );
      for(j=0; j<msg_checkers_.at(i).msg_from_motor_checkers.size();j++)
      {
        ROS_WARN_STREAM("    Motor checker id: " << msg_checkers_.at(i).msg_from_motor_checkers.at(j)->motor_id_ );
      }

    }
  }

  bool MotorDataChecker::check_message(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp,
                                       FROM_MOTOR_DATA_TYPE motor_data_type, int16u motor_slow_data_type)
  {
    int i;

    i = find(motor_data_type);
    if (i != (-1))
    {
      int j;
      j = msg_checkers_.at(i).find(joint_tmp->motor->motor_id);
      if (j != (-1))
      {
        if (motor_data_type == MOTOR_DATA_SLOW_MISC)
        {
          SlowMessageFromMotorChecker* ptr_tmp_checker = dynamic_cast<SlowMessageFromMotorChecker*>( msg_checkers_.at(i).msg_from_motor_checkers.at(j) );

          if(ptr_tmp_checker != NULL)
          {
            ROS_ERROR_STREAM("Slow iter ptr: " << ptr_tmp_checker);
            ptr_tmp_checker->set_received( static_cast<FROM_MOTOR_SLOW_DATA_TYPE>(motor_slow_data_type) );
          }
          else
          {
            ROS_ERROR_STREAM("CONVERSION FAILED");
          }
        }
        else
        {
          //we assume that the type of it2 is MessageFromMotorChecker
          msg_checkers_.at(i).msg_from_motor_checkers.at(j)->set_received();
        }
      }
      else
      {
        ROS_ERROR_STREAM("Motor id not found: " << joint_tmp->motor->motor_id );

        //TODO remove
            ROS_WARN_STREAM("Msg checkers size: " << msg_checkers_.size() );
            int i,j;
            for(i=0; i<msg_checkers_.size();i++)
            {
              ROS_WARN_STREAM("  Msg checker type: " << msg_checkers_.at(i).msg_type );
              ROS_WARN_STREAM("  Motor checker size: " << msg_checkers_.at(i).msg_from_motor_checkers.size() );
              for(j=0; j<msg_checkers_.at(i).msg_from_motor_checkers.size();j++)
              {
                ROS_WARN_STREAM("    Motor checker id: " << msg_checkers_.at(i).msg_from_motor_checkers.at(j)->motor_id_ );
              }

            }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Data type not found: " << motor_data_type );
    }
    return ((update_state == operation_mode::device_update_state::OPERATION) || is_everything_checked());
  }

  bool MotorDataChecker::is_everything_checked()
  {
    std::vector<MessageChecker>::iterator it;

    for (it = msg_checkers_.begin(); it < msg_checkers_.end(); it++)
    {
      std::vector<MessageFromMotorChecker*>::iterator it2;

      for (it2 = it->msg_from_motor_checkers.begin(); it2 < it->msg_from_motor_checkers.end(); it2++)
      {
        if (!(*it2)->get_received())
        {
          return false;
        }
      }
    }

    //all the motors are initialized -> we stop the timeout timer
    check_timeout_timer.stop();
    update_state = operation_mode::device_update_state::OPERATION;
    return true;
  }

  int MotorDataChecker::find(FROM_MOTOR_DATA_TYPE motor_data_type)
  {
    int i=0;
    for(;i<msg_checkers_.size();i++)
    {
      if (msg_checkers_.at(i).msg_type == motor_data_type)
        return i;
    }
    return (-1);
  }

  void MotorDataChecker::timer_callback(const ros::TimerEvent& event)
  {
    if( update_state == operation_mode::device_update_state::INITIALIZATION )
    {
      update_state = operation_mode::device_update_state::OPERATION;
      ROS_ERROR_STREAM("Motor Initialization Timeout: the static information in the diagnostics may not be uptodate.");
    }
  }

  int MessageChecker::find(int motor_id)
  {
    int i = 0;

    for (; i< msg_from_motor_checkers.size(); i++)
    {
      if (msg_from_motor_checkers.at(i)->motor_id_ == motor_id)
        return i;
    }
    return (-1);
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
    ROS_WARN_STREAM("actual. motor: " << motor_id_ << "  slow type received: " << slow_data_type);
    if (received_ == false)
    {
      //Check the slow data type as received
      if ( slow_data_type > MOTOR_SLOW_DATA_LAST )
      {
        ROS_ERROR_STREAM("Received bad slow_data_type: " << slow_data_type << " > " << slow_data_received.size() );
        return;
      }
      slow_data_received.at(slow_data_type) = true;

      ROS_ERROR_STREAM("motor: " << motor_id_ << "  slow type received: " << slow_data_type);

      //look if every type is received, then change FROM_MOTOR_SLOW_DATA_TYPE general received state accordingly
      bool checked = true;
      for (int i = MOTOR_SLOW_DATA_SVN_REVISION; i <= MOTOR_SLOW_DATA_LAST; i++)
      {
        checked = checked && slow_data_received.at(i);
        if (!checked)
        {
          ROS_INFO_STREAM(" still waiting for: " << i);
          break;
        }
      }
      if (checked)
        received_ = true;
    }
    else
    {
      ROS_WARN_STREAM("Already. motor: " << motor_id_ << "  slow type received: " << slow_data_type);
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
