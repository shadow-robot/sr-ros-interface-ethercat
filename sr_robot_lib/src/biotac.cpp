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

namespace tactiles
{
  Biotac::Biotac(std::vector<generic_updater::UpdateConfig> update_configs_vector)
    : GenericTactiles(update_configs_vector)
  {
    // Tactile sensor real time publisher
    tactile_publisher = boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll> >( new realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll>(nodehandle_ , "tactile", 4));

    //initialize the vector of tactiles
    tactiles_vector = boost::shared_ptr< std::vector<BiotacData> >( new std::vector<BiotacData>(nb_tactiles) );
  }

  void Biotac::update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    for( unsigned int id_sensor = 0; id_sensor < nb_tactiles; ++id_sensor)
    {
      switch( static_cast<int32u>(status_data->tactile_data_type) )
      {
        //TACTILE DATA
      case TACTILE_SENSOR_TYPE_BIOTAC_INVALID:
        ROS_WARN("received invalid tactile type");
        break;

      case TACTILE_SENSOR_TYPE_BIOTAC_PDC:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_TAC:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_TDC:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_1:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_2:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_3:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_4:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_5:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_6:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_7:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_8:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_9:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_10:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_11:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_12:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_13:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_14:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_15:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_16:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_17:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_18:
        break;
      case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_19:
        break;
      case FROM_TACTILE_SENSOR_TYPE_BIOTAC_NUM_VALUES = 0x0017
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

  void Biotac::publish()
  {
    if(tactile_publisher->trylock())
    {
      //for the time being, we only have PSTs tactile sensors
      sr_robot_msgs::BiotacAll tactiles;
      tactiles.header.stamp = ros::Time::now();

      //tactiles.pressure.push_back(sr_hand_lib->tactile_data_valid);

      for(unsigned int id_tact = 0; id_tact < nb_tactiles; ++id_tact)
      {
        sr_robot_msgs::Biotac tactile_tmp;
        if( tactiles_vector->at(id_tact).tactile_data_valid )
        {
          tactile_tmp.pdc = static_cast<int16u>(tactiles_vector->at(id_tact).pdc);
          tactile_tmp.tac = static_cast<int16u>(tactiles_vector->at(id_tact).tac);
          tactile_tmp.tdc = static_cast<int16u>(tactiles_vector->at(id_tact).tdc);

          tactile_tmp.electrodes = tactiles_vector->at(id_tact).electrodes;

          tactile_tmp.num_values = static_cast<int16u>(tactiles_vector->at(id_tact).num_values);
        }
        else
        {
          tactile_tmp.pdc = -1;
          tactile_tmp.tac = -1;
          tactile_tmp.tdc = -1;

          //TODO: push vector of -1 in electrodes?

          tactile_tmp.num_values = -1;
        }

        tactiles.tactiles.push_back(tactile_tmp);
      }


      tactile_publisher->msg_ = tactiles;
      tactile_publisher->unlockAndPublish();
    }

  }//end publish

  void Biotac::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                               diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for(unsigned int id_tact = 0; id_tact < nb_tactiles; ++id_tact)
    {
      std::stringstream ss;

      ss << "Tactile " << id_tact + 1;

      d.name = ss.str().c_str();
      d.summary(d.OK, "OK");
      d.clear();

      d.addf("Sample Frequency", "%d", tactiles_vector->at(id_tact).sample_frequency);
      d.addf("Manufacturer", "%s", tactiles_vector->at(id_tact).manufacturer.c_str());
      d.addf("Serial Number", "%s", tactiles_vector->at(id_tact).serial_number.c_str());

      d.addf("Software Version", "%d", tactiles_vector->at(id_tact).software_version);
      d.addf("PCB Version", "%d", tactiles_vector->at(id_tact).pcb_version);

      vec.push_back(d);
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


