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

#ifndef _MST_HPP_
#define _MST_HPP_

#include <vector>
#include <string>
#include <memory>
#include <sr_robot_msgs/MSTPalm.h>

#include "sr_robot_lib/generic_tactiles.hpp"
#include "sr_robot_lib/generic_updater.hpp"

namespace tactiles
{

template<class StatusType, class CommandType>
class MST :
        public GenericTactiles<StatusType, CommandType>
{
public:
  MST(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector);

  /*
   * This function is called each time a new etherCAT message
   * is received in the sr08.cpp driver. It  updates the tactile
   * sensors values contained in sensor_data.
   *
   * @param status_data the received etherCAT message
   */
  virtual void update(StatusType *status_data);

  /*
   * Publish the information to a ROS topic.
   *
   */
  virtual void publish();

  /*
   * This function adds the diagnostics for the tactiles to the
   * multi diagnostic status published by the hand.
   */
  virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                               diagnostic_updater::DiagnosticStatusWrapper &d);

private:
  sr_robot_msgs::MSTPalm sensor_data;
  std::shared_ptr<ros::Publisher> publisher;
  std::vector<GenericTactileData> diagnostic_data;

  int read12bits(char* buffer, int index);
};  // end class


// class template specialization.

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
};  // end class

}  // namespace tactiles

#endif  // _MST_HPP_
