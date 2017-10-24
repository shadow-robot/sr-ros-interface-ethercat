/**
 * @file   motor_updater.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Tue Jun  7 09:15:21 2011
 *
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
 * @brief  This contains a class used to determin which data we should ask the motor for,
 * depending on the config we're using.
 *
 *
 */

#include "sr_robot_lib/motor_updater.hpp"
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>

namespace generic_updater
{
  template<class CommandType>
  MotorUpdater<CommandType>::MotorUpdater(std::vector<UpdateConfig> update_configs_vector,
                                          operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericUpdater<CommandType>(update_configs_vector, update_state), even_motors(1)
  {
  }

  template<class CommandType>
  operation_mode::device_update_state::DeviceUpdateState MotorUpdater<CommandType>::build_init_command(
          CommandType *command)
  {
    if (!this->mutex->try_lock())
    {
      return this->update_state;
    }

    if (this->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      ///////
      // First we ask for the next data we want to receive
      if (even_motors)
      {
        even_motors = 0;
      }
      else
      {
        even_motors = 1;
        this->which_data_to_request++;

        if (this->which_data_to_request >= this->initialization_configs_vector.size())
        {
          this->which_data_to_request = 0;
        }
      }

      command->which_motors = even_motors;

      // initialization data
      command->from_motor_data_type =
              static_cast<FROM_MOTOR_DATA_TYPE>(
                      this->initialization_configs_vector[this->which_data_to_request].what_to_update);
      ROS_DEBUG_STREAM("Updating initialization data type: " << command->from_motor_data_type << " | [" <<
                       this->which_data_to_request << "/" << this->initialization_configs_vector.size() << "] ");
    }
    else
    {
      // For the last message sent when a change of update_state happens
      // (after that we use build_command instead of build_init_command)
      // we use the first important message and ask it to the even motors (0)
      // This is to avoid sending a random command
      command->which_motors = 0;
      command->from_motor_data_type =
              static_cast<FROM_MOTOR_DATA_TYPE>(this->important_update_configs_vector[0].what_to_update);
      ROS_DEBUG_STREAM("Updating important data type: " << command->from_motor_data_type << " | [" <<
                       this->which_data_to_request << "/" << this->important_update_configs_vector.size() << "] ");
    }

    this->mutex->unlock();

    return this->update_state;
  }

  template<class CommandType>
  operation_mode::device_update_state::DeviceUpdateState MotorUpdater<CommandType>::build_command(CommandType *command)
  {
    if (!this->mutex->try_lock())
    {
      return this->update_state;
    }

    ///////
    // First we ask for the next data we want to receive
    if (even_motors)
    {
      even_motors = 0;
    }
    else
    {
      even_motors = 1;
      this->which_data_to_request++;

      if (this->which_data_to_request >= this->important_update_configs_vector.size())
      {
        this->which_data_to_request = 0;
      }
    }

    command->which_motors = even_motors;

    if (!this->unimportant_data_queue.empty())
    {
      // an unimportant data is available
      command->from_motor_data_type = static_cast<FROM_MOTOR_DATA_TYPE>(this->unimportant_data_queue.front());
      this->unimportant_data_queue.pop();

      ROS_DEBUG_STREAM("Updating unimportant data type: " << command->from_motor_data_type << " | queue size: " <<
                       this->unimportant_data_queue.size());
    }
    else
    {
      // important data to update as often as possible
      command->from_motor_data_type =
              static_cast<FROM_MOTOR_DATA_TYPE>(
                      this->important_update_configs_vector[this->which_data_to_request].what_to_update);
      ROS_DEBUG_STREAM("Updating important data type: " << command->from_motor_data_type << " | [" <<
                       this->which_data_to_request << "/" << this->important_update_configs_vector.size() << "] ");
    }

    this->mutex->unlock();

    return this->update_state;
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class MotorUpdater<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class MotorUpdater<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class MotorUpdater<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;
}  // namespace generic_updater



/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
