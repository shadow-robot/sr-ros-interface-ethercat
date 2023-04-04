/* Copyright 2021, 2023 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/
/** 
  * @file MST.cpp
  *
  * @brief This is a class for accessing the data from the MST tactiles.
  */

#include "sr_robot_lib/MST.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#include <memory>
#include <string>
#include <vector>
#include <ros/console.h>


#define NUMBER_OF_TAXELS 17

namespace tactiles
{
  template<class StatusType, class CommandType>
  /**
    * MST object constructor.
    *
    */
  MST<StatusType, CommandType>::MST(ros::NodeHandle nh, std::string device_id,
                                      std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                      operation_mode::device_update_state::DeviceUpdateState update_state,
                                      boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector) :
    GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    diagnostic_data = *init_tactiles_vector;
    init(update_configs_vector, update_state);
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::init(std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                                 operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    // initialize the vector of tactiles
    this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >(
            new std::vector<AllTactileData>(1));
    this->all_tactile_data->at(0).type = "mst";

    for (uint8_t id_sensor; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      sensor_data.tactiles[id_sensor].magnetic_data.resize(NUMBER_OF_TAXELS);
      sensor_data.tactiles[id_sensor].temperature_data.resize(NUMBER_OF_TAXELS);
    }
  }

  template<class StatusType, class CommandType>
  int MST<StatusType, CommandType>::read12bits(char* buffer, int index)
  {
    int start = index * 3 / 2;
    if (index % 2 == 0)
    {
      int byte1 = buffer[start];
      int byte2 = buffer[start + 1];
      int b1 = (uint8_t)buffer[start];
      int b2 = (uint8_t)buffer[start+1];

      return byte1 << 4 | byte2 >> 4 & 0x0F;
    }
    int byte1 = (int8_t)(buffer[start] << 4);
    int byte2 = buffer[start + 1];
    int b2 = (uint8_t)buffer[start+1];

    return byte1 << 4 | byte2 & 0xFF;
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::update(StatusType *status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    for (unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      switch (static_cast<int32u>(status_data->tactile_data_type))
      {
        case TACTILE_SENSOR_TYPE_MST_MAGNETIC_INDUCTION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            for (uint8_t taxel_index = 0; taxel_index < NUMBER_OF_TAXELS; taxel_index++)
            {
              // Set a timestamp right before obtaining magnetic data
              sensor_data.tactiles[id_sensor].timestamp = ros::Time::now();
              geometry_msgs::Point taxel_magnetic_data;
              taxel_magnetic_data.x = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3);
              taxel_magnetic_data.y = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3 + 1);
              taxel_magnetic_data.z = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3 + 2);
              sensor_data.tactiles[id_sensor].magnetic_data[taxel_index] = taxel_magnetic_data;
            }
          }
          break;

        case TACTILE_SENSOR_TYPE_MST_TEMPERATURE:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            for (uint8_t taxel_index = 0; taxel_index < NUMBER_OF_TAXELS; taxel_index++)
            {
              char* tactile_data_pointer = status_data->tactile[id_sensor].string;
              // Set a timestamp right before obtaining temperature data
              sensor_data.tactiles[id_sensor].timestamp = ros::Time::now();
              sensor_data.tactiles[id_sensor].temperature_data[taxel_index] =
                  // +1 To skip PSoC temperature; converting reading to Celsius degrees
                  (read12bits(++tactile_data_pointer, taxel_index) - 1180) * 0.24 + 25;
            }
          }
          break;
      }
    }
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::publish()
  {
    // left empty, as this is published from the controller publisher
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                                      diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status_wrapper)
  {
    for (int id_sensor = 0; id_sensor < this->nb_tactiles; id_sensor++)
    {
      std::stringstream ss;
      std::string prefix = this->device_id_.empty() ? this->device_id_ : (this->device_id_ + " ");

      ss << prefix << "Tactile " << id_sensor + 1;

      diagnostic_status_wrapper.name = ss.str().c_str();
      diagnostic_status_wrapper.summary(diagnostic_status_wrapper.OK, "OK");
      diagnostic_status_wrapper.clear();

      diagnostic_status_wrapper.addf("Sample Frequency", "%d", diagnostic_data[id_sensor].sample_frequency);
      diagnostic_status_wrapper.addf("Manufacturer", "%s", diagnostic_data[id_sensor].manufacturer.c_str());
      diagnostic_status_wrapper.addf("Serial Number", "%s", diagnostic_data[id_sensor].serial_number.c_str());
      diagnostic_status_wrapper.addf("Software Version", "%s", diagnostic_data[id_sensor].git_revision.c_str());
      diagnostic_status_wrapper.addf("PCB Version", "%s", diagnostic_data[id_sensor].pcb_version.c_str());

      vec.push_back(diagnostic_status_wrapper);
    }
  }

  template<class StatusType, class CommandType>
  std::vector<AllTactileData> *MST<StatusType, CommandType>::get_tactile_data()
  {
    this->all_tactile_data->at(0).mst.sensor_data = sensor_data;

    return this->all_tactile_data.get();
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class MST<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class MST<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class MST<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;

  template
  class MST<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;

  template
  class MST<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND>;

}  // namespace tactiles
