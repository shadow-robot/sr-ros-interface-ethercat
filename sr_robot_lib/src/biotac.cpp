/**
 * @file   biotac.cpp
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

#include "sr_robot_lib/biotac.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#define TACTILE_DATA_LENGTH_BYTES TACTILE_DATA_LENGTH_BYTES_v1

namespace tactiles
{
  template <class StatusType, class CommandType>
  Biotac<StatusType, CommandType>::Biotac(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
    : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
  }

  template <class StatusType, class CommandType>
  Biotac<StatusType, CommandType>::Biotac(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state, boost::shared_ptr< std::vector<GenericTactileData> > init_tactiles_vector)
    : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
    tactiles_vector->clear();
    for(unsigned int i=0;i<this->nb_tactiles;i++)
    {
      BiotacData tmp_pst(init_tactiles_vector->at(i));
      tactiles_vector->push_back( tmp_pst );
    }
  }

  template <class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::init(std::vector<generic_updater::UpdateConfig> update_configs_vector, operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    // Tactile sensor real time publisher
    tactile_publisher = boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll> >( new realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll>(this->nodehandle_, "tactile", 4));

    //initialize the vector of tactiles
    tactiles_vector = boost::shared_ptr< std::vector<BiotacData> >( new std::vector<BiotacData>(this->nb_tactiles) );
    this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >( new std::vector<AllTactileData>(this->nb_tactiles) );
  }

  template <class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::update(StatusType* status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    for( unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      //We always receive pac0 and pac1
      tactiles_vector->at(id_sensor).pac0 = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
      tactiles_vector->at(id_sensor).pac1 = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[1]) );

      //the rest of the data is sampled at different rates
      switch( static_cast<int32u>(status_data->tactile_data_type) )
      {
        //TACTILE DATA
      case TACTILE_SENSOR_TYPE_BIOTAC_INVALID:
        ROS_WARN("received invalid tactile type");
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_PDC:
        tactiles_vector->at(id_sensor).pdc = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_TAC:
        tactiles_vector->at(id_sensor).tac = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_TDC:
        tactiles_vector->at(id_sensor).tdc = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_1:
        tactiles_vector->at(id_sensor).electrodes[0] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_2:
        tactiles_vector->at(id_sensor).electrodes[1] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_3:
        tactiles_vector->at(id_sensor).electrodes[2] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_4:
        tactiles_vector->at(id_sensor).electrodes[3] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_5:
        tactiles_vector->at(id_sensor).electrodes[4] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_6:
        tactiles_vector->at(id_sensor).electrodes[5] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_7:
        tactiles_vector->at(id_sensor).electrodes[6] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_8:
        tactiles_vector->at(id_sensor).electrodes[7] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_9:
        tactiles_vector->at(id_sensor).electrodes[8] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_10:
        tactiles_vector->at(id_sensor).electrodes[9] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_11:
        tactiles_vector->at(id_sensor).electrodes[10] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_12:
        tactiles_vector->at(id_sensor).electrodes[11] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_13:
        tactiles_vector->at(id_sensor).electrodes[12] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_14:
        tactiles_vector->at(id_sensor).electrodes[13] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_15:
        tactiles_vector->at(id_sensor).electrodes[14] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_16:
        tactiles_vector->at(id_sensor).electrodes[15] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_17:
        tactiles_vector->at(id_sensor).electrodes[16] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_18:
        tactiles_vector->at(id_sensor).electrodes[17] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_19:
        tactiles_vector->at(id_sensor).electrodes[18] = static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        break;

        //COMMON DATA
      case TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).sample_frequency = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]) );
        }
        break;

      case TACTILE_SENSOR_TYPE_MANUFACTURER:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).manufacturer = this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
        }
        break;

      case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
        if( sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor) )
        {
          tactiles_vector->at(id_sensor).serial_number = this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
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
          tactiles_vector->at(id_sensor).pcb_version = this->sanitise_string( status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES );
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
  void Biotac<StatusType, CommandType>::publish()
  {
    if(tactile_publisher->trylock())
    {
      //for the time being, we only have PSTs tactile sensors
      sr_robot_msgs::BiotacAll tactiles;
      tactiles.header.stamp = ros::Time::now();

      for(unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
      {
        sr_robot_msgs::Biotac tactile_tmp;

        tactile_tmp.pac0 = static_cast<int16u>(tactiles_vector->at(id_tact).pac0);
        tactile_tmp.pac1 = static_cast<int16u>(tactiles_vector->at(id_tact).pac1);

        tactile_tmp.pdc = static_cast<int16u>(tactiles_vector->at(id_tact).pdc);
        tactile_tmp.tac = static_cast<int16u>(tactiles_vector->at(id_tact).tac);
        tactile_tmp.tdc = static_cast<int16u>(tactiles_vector->at(id_tact).tdc);

        tactile_tmp.electrodes = tactiles_vector->at(id_tact).electrodes;

        tactiles.tactiles[id_tact] = tactile_tmp;
      }

      tactile_publisher->msg_ = tactiles;
      tactile_publisher->unlockAndPublish();
    }

  }//end publish

  template <class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                               diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for(unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
    {
      std::stringstream ss;
      std::string prefix = this->device_id_.empty() ? this->device_id_ : (this->device_id_ + " ");

      ss << prefix << "Tactile " << id_tact + 1;

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
  std::vector<AllTactileData>* Biotac<StatusType, CommandType>::get_tactile_data()
  {
    for( unsigned int i=0; i < tactiles_vector->size(); ++i)
      this->all_tactile_data->at(i).biotac = tactiles_vector->at(i);

    return this->all_tactile_data.get();
  }

  //Only to ensure that the template class is compiled for the types we are interested in
  template class Biotac<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class Biotac<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
  template class Biotac<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


