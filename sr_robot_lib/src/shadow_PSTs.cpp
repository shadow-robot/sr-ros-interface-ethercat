/**
 * @file   shadow_PSTs.cpp
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
 * @brief This is a class for accessing the data from the
 *        PSTs tactiles.
 *
 *
 */

#include "sr_robot_lib/shadow_PSTs.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#define TACTILE_DATA_LENGTH_BYTES TACTILE_DATA_LENGTH_BYTES_v1

namespace tactiles
{
  template <class StatusType, class CommandType>
  ShadowPSTs<StatusType, CommandType>::ShadowPSTs(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
    : GenericTactiles<StatusType, CommandType>(update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
  }

  template <class StatusType, class CommandType>
  ShadowPSTs<StatusType, CommandType>::ShadowPSTs(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state, boost::shared_ptr< std::vector<GenericTactileData> > init_tactiles_vector)
    : GenericTactiles<StatusType, CommandType>(update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
    tactiles_vector->clear();
    for(unsigned int i=0;i<this->nb_tactiles;i++)
    {
      PST3Data tmp_pst(init_tactiles_vector->at(i));
      tactiles_vector->push_back( tmp_pst );
    }
  }

  template <class StatusType, class CommandType>
  void ShadowPSTs<StatusType, CommandType>::init(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    // Tactile sensor real time publisher
    tactile_publisher = boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST> >( new realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST>(this->nodehandle_, "tactile", 4));

    //initialize the vector of tactiles
    tactiles_vector = boost::shared_ptr< std::vector<PST3Data> >( new std::vector<PST3Data>(this->nb_tactiles) );
    this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >( new std::vector<AllTactileData>(this->nb_tactiles) );
  }

  template <class StatusType, class CommandType>
  void ShadowPSTs<StatusType, CommandType>::update(StatusType* status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    for( unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      switch( static_cast<int32u>(status_data->tactile_data_type) )
      {
        //TACTILE DATA
      case TACTILE_SENSOR_TYPE_PST3_PRESSURE_TEMPERATURE:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).pressure = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          tactiles_vector->at(id_sensor).temperature = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[1]) );
          tactiles_vector->at(id_sensor).debug_1 = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
          tactiles_vector->at(id_sensor).debug_2 = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[3]) );
        }
        break;

      case TACTILE_SENSOR_TYPE_PST3_PRESSURE_RAW_ZERO_TRACKING:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).pressure_raw = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
          tactiles_vector->at(id_sensor).zero_tracking = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[1]) );
        }
        break;

      case TACTILE_SENSOR_TYPE_PST3_DAC_VALUE:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).dac_value = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
        }
        break;

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
          tactiles_vector->at(id_sensor).manufacturer = this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
      }
      break;

      case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
      {
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).serial_number = this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
      }
      break;

      case TACTILE_SENSOR_TYPE_SOFTWARE_VERSION:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).set_software_version( status_data->tactile[id_sensor].string );
        }
        break;

      case TACTILE_SENSOR_TYPE_PCB_VERSION:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).pcb_version =  this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
        break;

      default:
        break;

      } //end switch
    } //end for tactile

    if(this->sensor_updater->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      this->process_received_data_type(static_cast<int32u>(status_data->tactile_data_type));
      if(this->sensor_updater->initialization_configs_vector.size() == 0)
        this->sensor_updater->update_state = operation_mode::device_update_state::OPERATION;
    }
  }

  template <class StatusType, class CommandType>
  void ShadowPSTs<StatusType, CommandType>::publish()
  {
    if(tactile_publisher->trylock())
    {
      //for the time being, we only have PSTs tactile sensors
      sr_robot_msgs::ShadowPST tactiles;
      tactiles.header.stamp = ros::Time::now();

      //tactiles.pressure.push_back(sr_hand_lib->tactile_data_valid);

      for(unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
      {
        //Always publish the last valid data: the data are updated
        // only if they are valid
        tactiles.pressure.push_back( tactiles_vector->at(id_tact).pressure );
        tactiles.temperature.push_back( tactiles_vector->at(id_tact).temperature );
      }


      tactile_publisher->msg_ = tactiles;
      tactile_publisher->unlockAndPublish();
    }

  }//end publish

  template <class StatusType, class CommandType>
  void ShadowPSTs<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                   diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for(unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
    {
      std::stringstream ss;

      ss << "Tactile " << id_tact + 1;

      d.name = ss.str().c_str();
      d.summary(d.OK, "OK");
      d.clear();

      d.addf("Sample Frequency", "%d", tactiles_vector->at(id_tact).sample_frequency);
      d.addf("Manufacturer", "%s", tactiles_vector->at(id_tact).manufacturer.c_str());
      d.addf("Serial Number", "%s", tactiles_vector->at(id_tact).serial_number.c_str());

      d.addf("Software Version", "%s", tactiles_vector->at(id_tact).get_software_version().c_str());
      d.addf("PCB Version", "%s", tactiles_vector->at(id_tact).pcb_version.c_str());

      d.addf("Pressure Raw", "%d", tactiles_vector->at(id_tact).pressure_raw);
      d.addf("Zero Tracking", "%d", tactiles_vector->at(id_tact).zero_tracking);
      d.addf("DAC Value", "%d", tactiles_vector->at(id_tact).dac_value);

      vec.push_back(d);
    }
  }

  template <class StatusType, class CommandType>
  std::vector<AllTactileData>* ShadowPSTs<StatusType, CommandType>::get_tactile_data()
  {
    for( unsigned int i=0; i < tactiles_vector->size(); ++i)
      this->all_tactile_data->at(i).pst = tactiles_vector->at(i);

    return this->all_tactile_data.get();
  }

  //Only to ensure that the template class is compiled for the types we are interested in
  template class ShadowPSTs<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class ShadowPSTs<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
  template class ShadowPSTs<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


