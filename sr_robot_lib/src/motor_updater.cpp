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

namespace generic_updater
{
  MotorUpdater::MotorUpdater(std::vector<UpdateConfig> update_configs_vector, boost::shared_ptr<operation_mode::device_update_state::DeviceUpdateState> update_state)
    : GenericUpdater(update_configs_vector, update_state), even_motors(1)
  {
  }

  MotorUpdater::~MotorUpdater()
  {
  }

  void MotorUpdater::build_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command)
  {
    if(!mutex->try_lock())
      return;

    ///////
    // First we ask for the next data we want to receive
    if(even_motors)
      even_motors = 0;
    else
    {
      even_motors = 1;
      which_data_to_request ++;

      if( which_data_to_request >= important_update_configs_vector.size() )
        which_data_to_request = 0;
    }

    command->which_motors = even_motors;

    if(!unimportant_data_queue.empty())
    {
      //an unimportant data is available
      command->from_motor_data_type = static_cast<FROM_MOTOR_DATA_TYPE>(unimportant_data_queue.front());
      unimportant_data_queue.pop();

      ROS_DEBUG_STREAM("Updating unimportant data type: "<<command->from_motor_data_type << " | queue size: "<<unimportant_data_queue.size());
    }
    else
    {
      //important data to update as often as possible
      command->from_motor_data_type = static_cast<FROM_MOTOR_DATA_TYPE>(important_update_configs_vector[which_data_to_request].what_to_update);
      ROS_DEBUG_STREAM("Updating important data type: "<<command->from_motor_data_type << " | ["<<which_data_to_request<<"/"<<important_update_configs_vector.size()<<"] ");
    }

    mutex->unlock();
  }
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
