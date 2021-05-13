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

#define TACTILE_DATA_LENGTH_BYTES TACTILE_DATA_LENGTH_BYTES_v2

namespace tactiles
{
  template<class StatusType, class CommandType>
  MST<StatusType, CommandType>::MST(ros::NodeHandle nh, std::string device_id,
                                      std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                      operation_mode::device_update_state::DeviceUpdateState update_state,
                                      boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    publisher = std::make_shared<ros::Publisher>(nh.advertise<sr_robot_msgs::MSTPalm>("mst", 1));
    for (int i; i < this->nb_tactiles; i++)
    {
      diagnostic_data.push_back({"", "", "", -1, -1});
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
      // the rest of the data is sampled at different rates
      switch (static_cast<int32u>(status_data->tactile_data_type))
      {
        // TACTILE DATA
        case TACTILE_SENSOR_TYPE_MST_MAGNETIC_INDUCTION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            for (int i = 0; i < 7; i++)
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
            for (int i = 0; i < 7; i++)
            {
              sensor_data.fingers[id_sensor].sensors[i].temperature =
                status_data->tactile[id_sensor].string[i * 2] << 8 |
                (uint8_t)status_data->tactile[id_sensor].string[i * 2 + 1];
            }
          }
          break;

        // COMMON DATA
        case TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            diagnostic_data[id_sensor].sample_frequency =
              static_cast<int16u>(status_data->tactile[id_sensor].word[0]);
          }
          break;

        case TACTILE_SENSOR_TYPE_MANUFACTURER:
        {
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            diagnostic_data[id_sensor].manufacturer = this->sanitise_string(
              status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES);
          }
        }
          break;

        case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
        {
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            std::stringstream serial_number;
            for (int i = 0; i < 4; i++)
            {
              serial_number << std::hex << static_cast<int>(
                static_cast<uint8_t>(status_data->tactile[id_sensor].string[i]));
            }
            diagnostic_data[id_sensor].serial_number = serial_number.str();
            ROS_INFO_STREAM("MST serial number " << id_sensor << ": " << diagnostic_data[id_sensor].serial_number);
          }
        }
          break;

        case TACTILE_SENSOR_TYPE_SOFTWARE_VERSION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            std::stringstream git_revision;
            for (int i = 0; i < 20; i++)
            {
              git_revision << std::hex << static_cast<int>(
                static_cast<uint8_t>(status_data->tactile[id_sensor].string[i]));
            }
            diagnostic_data[id_sensor].git_revision = git_revision.str();
            ROS_INFO_STREAM("MST git revision " << id_sensor << ": " << diagnostic_data[id_sensor].git_revision);
          }
          break;

        case TACTILE_SENSOR_TYPE_PCB_VERSION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            diagnostic_data[id_sensor].pcb_version = status_data->tactile[id_sensor].string[0];
          }
          break;
      }
    }

    if (this->sensor_updater->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      this->process_received_data_type(static_cast<int32u>(status_data->tactile_data_type));
      if (this->sensor_updater->initialization_configs_vector.size() == 0)
      {
        this->sensor_updater->update_state = operation_mode::device_update_state::OPERATION;
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
      d.addf("PCB Version", "%d", diagnostic_data[i].pcb_version);

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
