/* Copyright 2021, 2023-2024 Shadow Robot Company Ltd.
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
  * @file  MST.hpp
  * @brief This is a class for accessing the data from the MST tactiles.
  */

#ifndef _MST_HPP_
#define _MST_HPP_

#include <vector>
#include <string>
#include <memory>
#include <sr_robot_msgs/MSTAll.h>

#include "sr_robot_lib/generic_tactiles.hpp"
#include "sr_robot_lib/generic_updater.hpp"

namespace tactiles
{
/** 
  * Class that contains all functions and member variables relevant to read the MST sensor data.
  * It inherits from the GenericTactiles class.
  */  
template<class StatusType, class CommandType>
class MST :
        public GenericTactiles<StatusType, CommandType>
{
public:
  /**
    * MST object constructor.
    *
    * @param nh ROS node handle.
    * @param device_id The device id.
    * @param update_configs_vector A vector containing information regarding which type of data to poll.
    * @param update_state Defines current operation state of the sensor (INITIALIZATION or OPERATION).
    * @param init_tactiles_vector A generic tactile data vector containing sensor information (e.g. Serial Number).
    */
  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector);

  /**
    * Initialise relevant MST sensor data arrays.
    */
  void initialise_tactile_data_structure(boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector);

  /**
    * Extract the tactile specific data from the incoming EtherCAT status message from the Palm.
    * And update the tactile data structure.
    *
    * @param status_data Is a pointer to an incoming EtherCAT status message from the Palm.
    */
  virtual void update(StatusType *status_data);

  /**
    * Publish MST tactile data. 
    * Not implemented, as this is published from the controller publisher.
    * But still necessary to keep it in the class, as it maybe be called by the hand drivers
    */
  virtual void publish();

  /**
    *  This function appends MST specific diagnostic data to the runtime_monitor node of a running Hand.
    *
    * @param diagnostic_vector The vector of diagnostic messages which to append the sensor's diagnostic data.
    * @param diagnostic_status_wrapper A diagnostic status wrapper used to update the sensor's diagnostics with.
    */
  virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &diagnostic_vector,
                               diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status_wrapper);

  /**
    * Getter for the current MST tactile data
    * 
    * @return A vector of tactiles::AllTactileData messages, updated with MST tactile data
    */
  virtual std::vector<AllTactileData> *get_tactile_data();

private:
  /// Stores current MST tactile data
  sr_robot_msgs::MSTAll sensor_data_;
  /// Stores MST diagnostics data
  std::vector<GenericTactileData> diagnostic_data_;
  /// Status Check "enable flag" found for each sensor
  int8_t status_check_byte_[5];

  /**
    * Decode incoming message from the MST tactile sensor, which are sent in chunks of 12 bits
    * Message is decoded for specified index of the data array. 
    *
    * @param buffer Is pointer to array containing incoming tactile data bytes.
    * @param index Is an index that corresponds the taxel (and channel - X, Y or Z) to be decoded.
    * @return The decoded integer value of the requested sensor's taxel and channel.
    */
  int read12bits(char* buffer, int index);
};  // end class

/*
* This is a specialization of the template class to avoid that the compiler try to access non existent fields,
* as it will compile the class for all the desired combinations of StatusType and CommandType,
* but in runtime this class will only be instantiated for those StatusType that actually contain MST sensor data
*/
template<>
class MST<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>
{
public:
  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  ~MST()
  {
  };
};  // end class

template<>
class MST<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>
{
public:
  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  ~MST()
  {
  };
};  // end class

template<>
class MST<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>
{
public:
  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_WARN("This MST tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE. "
             "Ignore this message if you are using a mockup tactile device.");
  }

  ~MST()
  {
  };
};
}  // namespace tactiles

#endif  // _MST_HPP_
