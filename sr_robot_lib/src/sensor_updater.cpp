/**
 * @file   sensor_updater.cpp
 * @author toni <toni@shadowrobot.com>
 * @date   20 Oct 2011
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
 * @brief This class is used to update the tactile sensor command.
 *
 *
 */


#include "sr_robot_lib/sensor_updater.hpp"
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/smart_ptr.hpp>
#include <iostream>
#include <vector>

namespace generic_updater
{
  template<class CommandType>
  SensorUpdater<CommandType>::SensorUpdater(std::vector<UpdateConfig> update_configs_vector,
                                            operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericUpdater<CommandType>(update_configs_vector, update_state)
  {
  }

  template<class CommandType>
  operation_mode::device_update_state::DeviceUpdateState SensorUpdater<CommandType>::build_init_command(
          CommandType *command)
  {
    if (!this->mutex->try_lock())
    {
      return this->update_state;
    }

    if (this->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      if (this->initialization_configs_vector.size() > 0)
      {
        ///////
        // First we ask for the next data we want to receive

        this->which_data_to_request++;

        if (this->which_data_to_request >= this->initialization_configs_vector.size())
        {
          this->which_data_to_request = 0;
        }

        // initialization data
        command->tactile_data_type = this->initialization_configs_vector[this->which_data_to_request].what_to_update;
        ROS_DEBUG_STREAM("Updating sensor initialization data type: " << command->tactile_data_type << " | [" <<
                         this->which_data_to_request << "/" << this->initialization_configs_vector.size() << "] ");
      }
    }
    else
    {
      // For the last message sent when a change of update_state happens
      // (after that we use build_command instead of build_init_command)
      // we use the TACTILE_SENSOR_TYPE_WHICH_SENSORS message, which is supposed to be always implemented
      // This is to avoid sending a random command (initialization_configs_vector is empty at this time)
      ROS_DEBUG_STREAM("Important data size: " << this->important_update_configs_vector.size());


      command->tactile_data_type = TACTILE_SENSOR_TYPE_WHICH_SENSORS;
      ROS_DEBUG_STREAM("Updating sensor initialization data type: " << command->tactile_data_type << " | [" <<
                       this->which_data_to_request << "/" << this->important_update_configs_vector.size() << "] ");
    }
    this->mutex->unlock();

    return this->update_state;
  }

  template<class CommandType>
  operation_mode::device_update_state::DeviceUpdateState SensorUpdater<CommandType>::build_command(CommandType *command)
  {
    if (!this->mutex->try_lock())
    {
      return this->update_state;
    }

    ///////
    // First we ask for the next data we want to receive

    this->which_data_to_request++;

    if (this->which_data_to_request >= this->important_update_configs_vector.size())
    {
      this->which_data_to_request = 0;
    }

    if (!this->unimportant_data_queue.empty())
    {
      // an unimportant data is available
      command->tactile_data_type = this->unimportant_data_queue.front();
      this->unimportant_data_queue.pop();

      ROS_DEBUG_STREAM("Updating sensor unimportant data type: " << command->tactile_data_type << " | queue size: " <<
                       this->unimportant_data_queue.size());
    }
    else
    {
      // important data to update as often as possible
      command->tactile_data_type = this->important_update_configs_vector[this->which_data_to_request].what_to_update;
      ROS_DEBUG_STREAM("Updating sensor important data type: " << command->tactile_data_type << " | [" <<
                       this->which_data_to_request << "/" << this->important_update_configs_vector.size() << "] ");
    }

    this->mutex->unlock();

    return this->update_state;
  }

  template<class CommandType>
  bool SensorUpdater<CommandType>::reset()
  {
    // We need to send the reset command twice in a row to make sure
    // the tactiles are reset.
    boost::mutex::scoped_lock l(*(this->mutex));
    for (unsigned int i = 0; i < 2; ++i)
    {
      this->unimportant_data_queue.push(TACTILE_SENSOR_TYPE_RESET_COMMAND);
    }
    return true;
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class SensorUpdater<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class SensorUpdater<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class SensorUpdater<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;

  template
  class SensorUpdater<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}  // namespace generic_updater




/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
