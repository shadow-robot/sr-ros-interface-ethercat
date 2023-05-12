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
  * @file   MST.cpp
  * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
  *         Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>,
  *         Dan Greenwald <dg@shadowrobot.com>, Rodrigo Zenha <rodrigo@shadowrobot.com>
  *         contact <software@shadowrobot.com>
  * @brief This is a class for accessing the data from the MST tactiles.
  */

#include "sr_robot_lib/MST.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#include <memory>
#include <string>
#include <vector>
#include <ros/console.h>

/// The number of sensitive taxels on each MST tactile sensor
#define NUMBER_OF_TAXELS 17

namespace tactiles
{
/**
  * MST object constructor.
  *
  * @param nh ROS node handle.
  * @param device_id The device id.
  * @param update_configs_vector A vector containing information regarding which type of data to poll.
  * @param update_state Defines current operation state of the sensor (INITIALIZATION or OPERATION).
  * @param init_tactiles_vector A generic tactile data vector containing sensor information (e.g. Serial Number).
  */
template<class StatusType, class CommandType>
MST<StatusType, CommandType>::MST(ros::NodeHandle nh, std::string device_id,
                                    std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                    operation_mode::device_update_state::DeviceUpdateState update_state,
                                    boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector) :
  GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
{
  diagnostic_data = *init_tactiles_vector;
  initialise_tactile_data_structure();
}

/**
  * Initialise relevant MST sensor data arrays.
  */
template<class StatusType, class CommandType>
void MST<StatusType, CommandType>::initialise_tactile_data_structure()
{
  // This is defined as an array of size 1, so that it's compatible with SrTactileSensorController::update()
  this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >(new std::vector<AllTactileData>(1));
  this->all_tactile_data->at(0).type = "mst";

  // Resize the tactile data structure to match the number of taxels on each sensor
  for (uint8_t id_sensor; id_sensor < this->nb_tactiles; ++id_sensor)
  {
    sensor_data.tactiles[id_sensor].magnetic_data.resize(NUMBER_OF_TAXELS);
    sensor_data.tactiles[id_sensor].temperature_data.resize(NUMBER_OF_TAXELS);
  }
}

/**
  * Decode incoming message from the MST tactile sensor, which are sent in chunks of 12 bits
  * Message is decoded for specified index of the data array. 
  *
  * @param buffer Is pointer to array containing incoming tactile data bytes.
  * @param index Is an index that corresponds the taxel (and channel - X, Y or Z) to be decoded.
  * @return The decoded integer value of the requested sensor's taxel and channel.
  */
template<class StatusType, class CommandType>
int MST<StatusType, CommandType>::read12bits(char* buffer, int index)
{
  // Finds the correct index of the data in the data buffer
  int start = index * 3 / 2;

  // If requested index is even, return the current byte, and the first 4 bits of the next byte
  if (index % 2 == 0)
  {
    int byte1 = buffer[start];
    int byte2 = buffer[start + 1];

    return byte1 << 4 | byte2 >> 4 & 0x0F;
  }
  // If requested index is odd, return the last 4 bits of the current byte, and the next byte
  int byte1 = (int8_t)(buffer[start] << 4);
  int byte2 = buffer[start + 1];

  return byte1 << 4 | byte2 & 0xFF;
}

/**
  * Extract the tactile specific data from the incoming EtherCAT status message from the Palm.
  * And update the tactile data structure.
  *
  * @param status_data Is a pointer to an incoming EtherCAT status message from the Palm.
  */
template<class StatusType, class CommandType>
void MST<StatusType, CommandType>::update(StatusType *status_data)
{
  // Returns a bit mask describing which fingers contain MST tactile sensors
  int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
  for (unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
  {
    // Assert the received MST data type
    switch (static_cast<int32u>(status_data->tactile_data_type))
    {
      case TACTILE_SENSOR_TYPE_MST_MAGNETIC_INDUCTION:
        // Check if the current sensor (id_sensor) is present in the bit mask
        if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
        {
          for (uint8_t taxel_index = 0; taxel_index < NUMBER_OF_TAXELS; taxel_index++)
          {
            // Set a timestamp right before extracting magnetic data
            sensor_data.tactiles[id_sensor].timestamp = ros::Time::now();
            geometry_msgs::Point taxel_magnetic_data;
            // Decode incoming tactile data for each taxel and channel (X, Y or Z)
            // And build ROS message
            taxel_magnetic_data.x = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3);
            taxel_magnetic_data.y = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3 + 1);
            taxel_magnetic_data.z = read12bits(status_data->tactile[id_sensor].string, taxel_index * 3 + 2);
            sensor_data.tactiles[id_sensor].magnetic_data[taxel_index] = taxel_magnetic_data;
          }
          if (diagnostic_data[id_sensor].git_revision.back() == '1')
            sensor_data.tactiles[id_sensor].status = (int8_t)status_data->tactile[id_sensor].string[(taxel_index-1) * 3 + 2] & 0x0F;
          else
            sensor_data.tactiles[id_sensor].status = -1;

        }
        break;

      case TACTILE_SENSOR_TYPE_MST_TEMPERATURE:
        // Check if the current sensor (id_sensor) is present in the bit mask
        if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
        {
          for (uint8_t taxel_index = 0; taxel_index < NUMBER_OF_TAXELS; taxel_index++)
          {
            char* tactile_data_pointer = status_data->tactile[id_sensor].string;
            // Set a timestamp right before extracting temperature data
            sensor_data.tactiles[id_sensor].timestamp = ros::Time::now();
            // Decode incoming tactile data for each taxel; Skiping first byte, as it contains MST's PSoC temperature
            sensor_data.tactiles[id_sensor].temperature_data[taxel_index] =
                (read12bits(++tactile_data_pointer, taxel_index) - 1180) * 0.24 + 25;  // Converts to Celsius degrees
          }
          if (diagnostic_data[id_sensor].git_revision.back() == '1')
            sensor_data.tactiles[id_sensor].status = (int8_t)status_data->tactile[id_sensor].string[(taxel_index-1)] & 0x0F;
          else
            sensor_data.tactiles[id_sensor].status = -1;
        }
        break;
    }
  }
}

/**
  * Publish MST tactile data. 
  * Not implemented, as this is published from the controller publisher.
  * But still necessary to keep it in the class, as it maybe be called by the hand drivers
  */
template<class StatusType, class CommandType>
void MST<StatusType, CommandType>::publish()
{
  // Left empty, as this is published from the controller publisher
}

/**
  *  This function appends MST specific diagnostic data to the runtime_monitor node of a running Hand.
  *
  * @param diagnostic_vector The vector of diagnostic messages which to append the sensor's diagnostic data.
  * @param diagnostic_status_wrapper A diagnostic status wrapper, used to update the sensor's diagnostics with
  */
template<class StatusType, class CommandType>
void MST<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &diagnostic_vector,
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

    diagnostic_vector.push_back(diagnostic_status_wrapper);
  }
}

/**
  * Getter for the current MST tactile data
  * 
  * @return A vector of tactiles::AllTactileData messages, updated with MST tactile data
  */
template<class StatusType, class CommandType>
std::vector<AllTactileData> *MST<StatusType, CommandType>::get_tactile_data()
{
  // sensor_data contains the data for all available tactile sensors
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
