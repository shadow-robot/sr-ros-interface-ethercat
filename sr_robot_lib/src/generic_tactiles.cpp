/**
 * @file   generic_tactiles.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Th Oct 20 10:06:14 2011
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
 * @brief This is the main class for accessing the data from the
 *        tactiles.
 *
 *
 */

#include "sr_robot_lib/generic_tactiles.hpp"
#include <sr_utilities/sr_math_utils.hpp>

namespace tactiles
{
  const unsigned int GenericTactiles::nb_tactiles = 5;

  GenericTactiles::GenericTactiles(std::vector<generic_updater::UpdateConfig> update_configs_vector, boost::shared_ptr<operation_mode::device_update_state::DeviceUpdateState> update_state)
    :update_state(update_state)
  {
    sensor_updater = boost::shared_ptr<generic_updater::SensorUpdater>(new generic_updater::SensorUpdater(update_configs_vector, update_state));
    reset_service_client_ = nodehandle_.advertiseService("/tactiles/reset", &GenericTactiles::reset, this);
  }

  void GenericTactiles::update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
    {
      int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
      //TODO: use memcopy instead?
      for( unsigned int id_sensor = 0; id_sensor < nb_tactiles; ++id_sensor)
      {
        switch( static_cast<int32u>(status_data->tactile_data_type) )
        {
          //COMMON DATA
        case TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ:
          if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
          {
            tactiles_vector->at(id_sensor).sample_frequency = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          }
          break;

        case TACTILE_SENSOR_TYPE_MANUFACTURER:
        {
          if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
          {
            std::string manufacturer = "";
            for (int i = 0; i < 16; ++i)
            {
              char tmp = static_cast<char>(status_data->tactile[id_sensor].string[i]);
              if( tmp != '0' )
                manufacturer += static_cast<char>(status_data->tactile[id_sensor].string[i]);
              else
                break;
            }
            tactiles_vector->at(id_sensor).manufacturer = manufacturer;
          }
        }
        break;

        case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
        {
          if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
          {
            std::string serial = "";
            for (int i = 0; i < 16; ++i)
            {
              char tmp = static_cast<char>(status_data->tactile[id_sensor].string[i]);
              if( tmp != 0 )
                serial += static_cast<char>(status_data->tactile[id_sensor].string[i]);
              else
                break;
            }
            tactiles_vector->at(id_sensor).serial_number = serial;
          }
        }
        break;

        case TACTILE_SENSOR_TYPE_SOFTWARE_VERSION:
          if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
          {
            tactiles_vector->at(id_sensor).software_version = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          }
          break;

        case TACTILE_SENSOR_TYPE_PCB_VERSION:
          if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
          {
            tactiles_vector->at(id_sensor).pcb_version = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          }
          break;

        default:
          break;

        } //end switch
      } //end for tactile
    }

  /**
   * Reset the tactile sensors.
   *
   * @param request empty
   * @param response empty
   *
   * @return true if success
   */
  bool GenericTactiles::reset(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
  {
    ROS_INFO_STREAM("Resetting tactiles");

    return sensor_updater->reset();
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


