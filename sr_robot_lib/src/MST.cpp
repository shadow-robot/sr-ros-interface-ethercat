/* Copyright 2021 Shadow Robot Company Ltd.
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
*
*
* @brief This is a class for accessing the data from the MST tactiles.
*/

#include "sr_robot_lib/MST.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#include <memory>
#include <string>
#include <vector>

#define NUMBER_OF_SENSORS 7

namespace tactiles
{
  template<class StatusType, class CommandType>
  MST<StatusType, CommandType>::MST(ros::NodeHandle nh, std::string device_id,
                                      std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                      operation_mode::device_update_state::DeviceUpdateState update_state,
                                      boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    diagnostic_data = *init_tactiles_vector;
    publisher = std::make_shared<ros::Publisher>(nh.advertise<sr_robot_msgs::MSTPalm>("mst", 1));
  }

  template<class StatusType, class CommandType>
  int MST<StatusType, CommandType>::read12bits(char* buffer, int index)
  {
    int start = index * 3 / 2;
    if (index % 2 == 0)
    {
      int byte1 = buffer[start];
      int byte2 = buffer[start + 1];
      return byte1 << 4 | byte2 >> 4 & 0x0F;
    }
    int byte1 = (int8_t)(buffer[start] << 4);
    int byte2 = buffer[start + 1];
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
            for (int i = 0; i < NUMBER_OF_SENSORS; i++)
            {
              sensor_data.fingers[id_sensor].sensors[i].magnetic_induction_x = read12bits(
                status_data->tactile[id_sensor].string, i * 3);
              sensor_data.fingers[id_sensor].sensors[i].magnetic_induction_y = read12bits(
                status_data->tactile[id_sensor].string, i * 3 + 1);
              sensor_data.fingers[id_sensor].sensors[i].magnetic_induction_z = read12bits(
                status_data->tactile[id_sensor].string, i * 3 + 2);
            }
          }
          break;

        case TACTILE_SENSOR_TYPE_MST_TEMPERATURE:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            for (int i = 0; i < NUMBER_OF_SENSORS; i++)
            {
              // Temperature is send as little-endian value
              sensor_data.fingers[id_sensor].sensors[i].temperature =
                status_data->tactile[id_sensor].string[i * 2 + 1] << 8 |
                (uint8_t)status_data->tactile[id_sensor].string[i * 2];
            }
          }
          break;
      }
    }
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::publish()
  {
    sensor_data.header.stamp = ros::Time::now();
    publisher->publish(sensor_data);
  }

  template<class StatusType, class CommandType>
  void MST<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                                      diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for (int i = 0; i < this->nb_tactiles; i++)
    {
      std::stringstream ss;
      std::string prefix = this->device_id_.empty() ? this->device_id_ : (this->device_id_ + " ");

      ss << prefix << "Tactile " << i + 1;

      d.name = ss.str().c_str();
      d.summary(d.OK, "OK");
      d.clear();

      d.addf("Sample Frequency", "%d", diagnostic_data[i].sample_frequency);
      d.addf("Manufacturer", "%s", diagnostic_data[i].manufacturer.c_str());
      d.addf("Serial Number", "%s", diagnostic_data[i].serial_number.c_str());
      d.addf("Software Version", "%s", diagnostic_data[i].git_revision.c_str());
      d.addf("PCB Version", "%s", diagnostic_data[i].pcb_version.c_str());

      vec.push_back(d);
    }
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

}  // namespace tactiles
