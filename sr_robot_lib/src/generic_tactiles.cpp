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
#include <cctype>

namespace tactiles
{
  const unsigned int GenericTactiles::nb_tactiles = 5;

  GenericTactiles::GenericTactiles(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    sensor_updater = boost::shared_ptr<generic_updater::SensorUpdater>(new generic_updater::SensorUpdater(update_configs_vector, update_state));
    if(update_state != operation_mode::device_update_state::INITIALIZATION)
    {
      reset_service_client_ = nodehandle_.advertiseService("/tactiles/reset", &GenericTactiles::reset, this);
    }

    //initialize the vector of tactiles
    tactiles_vector = boost::shared_ptr< std::vector<GenericTactileData> >( new std::vector<GenericTactileData>(nb_tactiles) );
    all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >( new std::vector<AllTactileData>(nb_tactiles) );

    for(unsigned int i=0;i<nb_tactiles;i++)
    {
      GenericTactileData tmp_pst;
      tactiles_vector->push_back( tmp_pst );
    }
  }


  void GenericTactiles::update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    for( unsigned int id_sensor = 0; id_sensor < nb_tactiles; ++id_sensor)
    {
      ROS_DEBUG_STREAM(" received: " << static_cast<int32u>(status_data->tactile_data_type));

      switch( static_cast<int32u>(status_data->tactile_data_type) )
      {
      //COMMON DATA
      case TACTILE_SENSOR_TYPE_WHICH_SENSORS:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          if( tactiles_vector != NULL )
            tactiles_vector->at(id_sensor).which_sensor = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          ROS_DEBUG_STREAM(" tact["<<id_sensor<<"] = " << tactiles_vector->at(id_sensor).which_sensor);

        }
        break;

      case TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          if( tactiles_vector != NULL )
            tactiles_vector->at(id_sensor).sample_frequency = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
        }
        break;

      case TACTILE_SENSOR_TYPE_MANUFACTURER:
      {
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).manufacturer = sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
      }
      break;

      case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
      {
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).serial_number = sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
      }
      break;

      case TACTILE_SENSOR_TYPE_SOFTWARE_VERSION:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          if( tactiles_vector != NULL )
            tactiles_vector->at(id_sensor).set_software_version( status_data->tactile[id_sensor].string );
        }
        break;

      case TACTILE_SENSOR_TYPE_PCB_VERSION:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          if( tactiles_vector != NULL )
            tactiles_vector->at(id_sensor).pcb_version = sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
        break;

      default:
        break;

      } //end switch
    } //end for tactile

    if(sensor_updater->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      process_received_data_type(static_cast<int32u>(status_data->tactile_data_type));
      if(sensor_updater->initialization_configs_vector.size() == 0)
        sensor_updater->update_state = operation_mode::device_update_state::OPERATION;
    }
  }

  void GenericTactiles::process_received_data_type(int32u data)
  {
    unsigned int i;
    for(i=0; i<sensor_updater->initialization_configs_vector.size(); i++)
    {
      if (sensor_updater->initialization_configs_vector[i].what_to_update == data) break;
    }
    if(i<sensor_updater->initialization_configs_vector.size())
      sensor_updater->initialization_configs_vector.erase(sensor_updater->initialization_configs_vector.begin() + i);
  }

  void GenericTactiles::publish()
  {
    //We don't publish anything during the initialization phase
  }//end publish

  void GenericTactiles::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                        diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    //We don't publish diagnostics during the initialization phase
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


  std::string GenericTactiles::sanitise_string( const char* raw_string, const unsigned int str_size )
  {
    std::string sanitised_string = "";
    for (unsigned int i = 0; i < str_size; ++i)
    {
      char tmp = static_cast<char>( raw_string[i] );
      if( tmp != 0 )
      {
        if( tmp >= '\x20' && tmp <= '\x7E')
        {
          sanitised_string += static_cast<char>( raw_string[i] ) ;
        }
        else
          sanitised_string += '?';
      }
      else
        break;
    }
    return sanitised_string;
  }

  std::vector<AllTactileData>* GenericTactiles::get_tactile_data()
  {
    return all_tactile_data.get();
  }

}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


