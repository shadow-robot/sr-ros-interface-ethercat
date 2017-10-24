/**
 * @file   sensor_updater.hpp
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

#ifndef SENSOR_UPDATER_HPP_
#define SENSOR_UPDATER_HPP_


#include <ros/ros.h>
#include <vector>
#include <list>
#include <queue>
#include "sr_robot_lib/generic_updater.hpp"

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
#include <sr_external_dependencies/external/0240_palm_edc_IMU/0240_palm_edc_IMU_ethercat_protocol.h>
}

namespace generic_updater
{
/**
 * The Sensor Updater builds the next command we want to send to the hand.
 * We can ask for different types of data at different rates. The data and
 * their rates are defined in the sr_ethercat_hand_config/rates/sensor_data_polling.yaml
 * The important data are refreshed as often as possible (they have a -1. refresh
 * rate in the config file).
 *
 * The unimportant data are refreshed at their given rate (the value is defined in
 * the config in seconds).
 */
template<class CommandType>
class SensorUpdater :
        public GenericUpdater<CommandType>
{
public:
  SensorUpdater(std::vector<UpdateConfig> update_configs_vector,
                operation_mode::device_update_state::DeviceUpdateState update_state);

  /**
   * Updates the initialization command to send to the hand. This function is called
   * at each packCommand() call. Ask for the relevant information for the tactiles.
   *
   * @param command The command which will be sent to the palm.
   * @return current update state
   */
  operation_mode::device_update_state::DeviceUpdateState build_init_command(CommandType *command);

  /**
   * Updates the command to send to the hand. This function is called
   * at each packCommand() call. Ask for the relevant information for the tactiles.
   * If an unimportant data is waiting then we send it, otherwise, we send the next
   * important data.
   *
   * @param command The command which will be sent to the palm.
   * @return current update state
   */
  operation_mode::device_update_state::DeviceUpdateState build_command(CommandType *command);

  /**
   * Will send the reset command to the tactiles, on next build
   * command call.
   *
   * Simply adds the reset command to the unimportant_data_queue.
   *
   *
   * @return true if RESET added to the top queue.
   */
  bool reset();
};
}  // namespace generic_updater


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* SENSOR_UPDATER_HPP_ */
