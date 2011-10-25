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
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 *
 *
 */

#include "sr_robot_lib/motor_data_checker.hpp"

namespace generic_updater
{

MotorDataChecker::MotorDataChecker(boost::ptr_vector<shadow_joints::Joint> joints_vector,
                                   std::vector<UpdateConfig> initialization_configs_vector)
{
  std::vector<UpdateConfig>::iterator msg_it;

  for (msg_it = initialization_configs_vector.begin(); msg_it < initialization_configs_vector.end(); msg_it++)
  {
    boost::ptr_vector<shadow_joints::Joint>::iterator joint;
    for (joint = joints_vector.begin(); joint < joints_vector.end(); joint++)
      {

      }
  }

}

bool MotorDataChecker::check_message(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp,
                                     FROM_MOTOR_DATA_TYPE motor_data_type,
                                     FROM_MOTOR_SLOW_DATA_TYPE motor_slow_data_type)
{
  std::vector<MessageChecker>::iterator it;

  it = find(motor_data_type);
  if (it != NULL)
  {
    std::vector<MessageFromMotorChecker>::iterator it2;
    it2 = it->find(joint_tmp->motor->motor_id);
    if (it2 != NULL)
    {
      if (motor_data_type == MOTOR_DATA_SLOW_MISC)
      {
        //we assume that the type of it2 is SlowMessageFromMotorChecker
        static_cast<SlowMessageFromMotorChecker*>(it2)->set_received(motor_slow_data_type);
      }
      else
      {
        //we assume that the type of it2 is MessageFromMotorChecker
        it2->set_received();
      }
    }
  }

  return is_everything_checked();
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
  return NULL;
}

std::vector<MessageFromMotorChecker>::iterator MessageChecker::find(int motor_id)
{
  std::vector<MessageFromMotorChecker>::iterator it;

  for (it = msg_from_motor_checkers.begin(); it < msg_from_motor_checkers.end(); it++)
  {
    if (it->motor_id_ == motor_id)
      return it;
  }
  return NULL;
}

SlowMessageFromMotorChecker::SlowMessageFromMotorChecker(int id) :
    MessageFromMotorChecker(id)
{
  for (int i = 0; i < MOTOR_SLOW_DATA_LAST + 1; i++)
  {
    slow_data_received[i] = false;
  }
}

void SlowMessageFromMotorChecker::set_received(FROM_MOTOR_SLOW_DATA_TYPE slow_data_type)
{
  if (received_ == false)
  {
    //Check the slow data type as received
    slow_data_received[slow_data_type] = true;

    //look if every type is received, then change FROM_MOTOR_SLOW_DATA_TYPE general received state accordingly
    bool checked = true;
    for (int i = MOTOR_SLOW_DATA_SVN_REVISION; i <= MOTOR_SLOW_DATA_LAST; i++)
    {
      checked &= slow_data_received[i];
      if (!checked)
        break;
    }
    if (checked)
      received_ = true;
  }
}

MessageFromMotorChecker::MessageFromMotorChecker(int id) :
    motor_id_(id), received_(false)
{
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
