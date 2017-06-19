/**
 * @file   muscle_updater.hpp
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

#ifndef _MUSCLE_UPDATER_HPP_
#define _MUSCLE_UPDATER_HPP_

#include <ros/ros.h>
#include <vector>
#include <list>
#include <queue>
#include "sr_robot_lib/generic_updater.hpp"

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/0320_palm_edc_muscle/0320_palm_edc_ethercat_protocol.h>
}

namespace generic_updater
{
/**
 * The Motor Updater builds the next command we want to send to the hand.
 * We can ask for different types of data at different rates. The data and
 * their rates are defined in the sr_ethercat_hand_config/rates/motor_data_polling.yaml
 * The important data are refreshed as often as possible (they have a -1. refresh
 * rate in the config file).
 *
 * The unimportant data are refreshed at their given rate (the value is defined in
 * the config in seconds).
 */
template<class CommandType>
class MuscleUpdater :
        public GenericUpdater<CommandType>
{
public:
  MuscleUpdater(std::vector<UpdateConfig> update_configs_vector,
                operation_mode::device_update_state::DeviceUpdateState update_state);

  /**
   * Building the motor initialization command. This function is called at each packCommand() call.
   * It builds initialization commands if the update state is operation_mode::device_update_state::INITIALIZATION.
   *
   * @param command The command which will be sent to the motor.
   * @return the current update state of the motor update
   */
  operation_mode::device_update_state::DeviceUpdateState build_init_command(CommandType *command);

  /**
   * Building the motor command. This function is called at each packCommand() call.
   * If an unimportant data is waiting then we send it, otherwise, we send the next
   * important data.
   *
   * @param command The command which will be sent to the motor.
   * @return the current update state of the motor update
   */
  operation_mode::device_update_state::DeviceUpdateState build_command(CommandType *command);
};
}  // namespace generic_updater


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
