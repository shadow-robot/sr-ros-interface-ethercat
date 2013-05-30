/**
 * @file   UBI0.cpp
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
 *        Biotac tactiles.
 *
 *
 */

#include "sr_robot_lib/UBI0.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#define TACTILE_DATA_LENGTH_BYTES TACTILE_DATA_LENGTH_BYTES_v2

namespace tactiles
{
  template <class StatusType, class CommandType>
  UBI0<StatusType, CommandType>::UBI0(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
    : GenericTactiles<StatusType, CommandType>(update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
  }

  template <class StatusType, class CommandType>
  UBI0<StatusType, CommandType>::UBI0(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state, boost::shared_ptr< std::vector<GenericTactileData> > init_tactiles_vector)
    : GenericTactiles<StatusType, CommandType>(update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
    tactiles_vector->clear();
    for(unsigned int i=0;i<this->nb_tactiles;i++)
    {
      UBI0Data tmp_pst(init_tactiles_vector->at(i));
      tactiles_vector->push_back( tmp_pst );
    }
  }

  template <class StatusType, class CommandType>
  void UBI0<StatusType, CommandType>::init(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    // Tactile sensor real time publisher
    tactile_publisher = boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All> >( new realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All>(this->nodehandle_ , "tactile", 4));

    //initialize the vector of tactiles
    tactiles_vector = boost::shared_ptr< std::vector<UBI0Data> >( new std::vector<UBI0Data>(this->nb_tactiles) );
    this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >( new std::vector<AllTactileData>(this->nb_tactiles) );

    //initialize the palm sensors
    palm_tactiles = boost::shared_ptr< UBI0PalmData >( new UBI0PalmData() );
  }

  template <class StatusType, class CommandType>
  void UBI0<StatusType, CommandType>::update(StatusType* status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    for( unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      //the rest of the data is sampled at different rates
      switch( static_cast<int32u>(status_data->tactile_data_type) )
      {
        //TACTILE DATA
      case TACTILE_SENSOR_TYPE_UBI0_TACTILE:
        for( unsigned int i = 0; i < tactiles_vector->at(id_sensor).distal.size(); ++i)
        {
          tactiles_vector->at(id_sensor).distal[i] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[i]) );
        }
        for( unsigned int i = 0; i < tactiles_vector->at(id_sensor).middle.size(); ++i)
        {
          tactiles_vector->at(id_sensor).middle[i] = static_cast<int>(static_cast<int16u>(status_data->tactile_mid_prox[id_sensor].middle[i]) );
        }
        for( unsigned int i = 0; i < tactiles_vector->at(id_sensor).proximal.size(); ++i)
        {
          tactiles_vector->at(id_sensor).proximal[i] = static_cast<int>(static_cast<int16u>(status_data->tactile_mid_prox[id_sensor].proximal[i]) );
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
          tactiles_vector->at(id_sensor).set_software_version( status_data->tactile[id_sensor].string );
        }
        break;

      case TACTILE_SENSOR_TYPE_PCB_VERSION:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).pcb_version = sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
        break;

      default:
        break;

      } //end switch
    } //end for tactile

    for( unsigned int i = 0; i < palm_tactiles->palm.size(); ++i)
    {
      palm_tactiles->palm[i] = static_cast<int>(static_cast<int16u>(status_data->tactile_palm.sensor[i]) );
    }

    if(this->sensor_updater->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      this->process_received_data_type(static_cast<int32u>(status_data->tactile_data_type));
      if(this->sensor_updater->initialization_configs_vector.size() == 0)
        this->sensor_updater->update_state = operation_mode::device_update_state::OPERATION;
    }
  }

  template <class StatusType, class CommandType>
  void UBI0<StatusType, CommandType>::publish()
  {
    if(tactile_publisher->trylock())
    {
      sr_robot_msgs::UBI0All tactiles;
      tactiles.header.stamp = ros::Time::now();

      //tactiles.pressure.push_back(sr_hand_lib->tactile_data_valid);

      for(unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
      {
        sr_robot_msgs::UBI0 tactile_tmp;
        //if( tactiles_vector->at(id_tact).tactile_data_valid )
        //{

        tactile_tmp.distal = tactiles_vector->at(id_tact).distal;
        tactile_tmp.middle = tactiles_vector->at(id_tact).middle;
        tactile_tmp.proximal = tactiles_vector->at(id_tact).proximal;

        //}
        //else
        //{
        //  tactile_tmp.pac0 = -1;
        //  tactile_tmp.pac1 = -1;
          //TODO: push vector of -1 in electrodes?
        //}

        tactiles.finger_tactiles[id_tact] = tactile_tmp;
      }

      tactiles.palm_tactiles.sensors = palm_tactiles->palm;


      tactile_publisher->msg_ = tactiles;
      tactile_publisher->unlockAndPublish();
    }

  }//end publish

  template <class StatusType, class CommandType>
  void UBI0<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
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

      vec.push_back(d);
    }
  }

  template <class StatusType, class CommandType>
  std::vector<AllTactileData>* UBI0<StatusType, CommandType>::get_tactile_data()
  {
    for( unsigned int i=0; i < tactiles_vector->size(); ++i)
      this->all_tactile_data->at(i).ubi0 = tactiles_vector->at(i);

    return this->all_tactile_data.get();
  }

  //Only to ensure that the template class is compiled for the types we are interested in
  template class UBI0<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class UBI0<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
  template class UBI0<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
