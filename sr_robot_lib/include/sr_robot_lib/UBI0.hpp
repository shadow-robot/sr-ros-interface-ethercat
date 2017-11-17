/**
 * @file   UBI0.hpp
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

#ifndef _UBI0_HPP_
#define _UBI0_HPP_

#include <vector>
#include <string>
#include <sr_robot_msgs/UBI0All.h>
#include <sr_robot_msgs/UBI0.h>
#include <sr_robot_msgs/AuxSpiData.h>
#include <sr_robot_msgs/MidProxDataAll.h>
#include <sr_robot_msgs/MidProxData.h>
#include <realtime_tools/realtime_publisher.h>

#include "sr_robot_lib/generic_tactiles.hpp"
#include "sr_robot_lib/generic_updater.hpp"

namespace tactiles
{
template<class StatusType, class CommandType>
class UBI0 :
        public GenericTactiles<StatusType, CommandType>
{
public:
  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state);

  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector);

  /**
   * This function is called in the constructors, to initialize the necessary objects
   */
  void init(std::vector<generic_updater::UpdateConfig> update_configs_vector,
            operation_mode::device_update_state::DeviceUpdateState update_state);

  /**
   * This function is called each time a new etherCAT message
   * is received in the sr06.cpp driver. It  updates the tactile
   * sensors values contained in tactiles_vector.
   *
   * @param status_data the received etherCAT message
   */
  virtual void update(StatusType *status_data);

  /**
   * Publish the information to a ROS topic.
   *
   */
  virtual void publish();

  /**
   * This function adds the diagnostics for the tactiles to the
   * multi diagnostic status published by the hand.
   */
  virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                               diagnostic_updater::DiagnosticStatusWrapper &d);


  virtual std::vector<AllTactileData> *get_tactile_data();

protected:
  /// the vector containing the data for the tactiles.
  boost::shared_ptr<std::vector<UBI0Data> > tactiles_vector;

  /// the object containing the data from the palm sensors
  boost::shared_ptr<UBI0PalmData> palm_tactiles;

  // Auxiliar Spi data (sometimes it is a palm tactile sensor) real time publisher
  boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::AuxSpiData> > aux_spi_publisher;
};  // end class


// class template specialization.

/**
 * This is a specialization of the template class to avoid that the compiler try to access non existent fields,
 * as it will compile the class for all the desired combinations of StatusType and CommandType,
 * but in runtime this class will only be instantiated for those StatusType that actually contain UBI0 sensor data
 */
template<>
class UBI0<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>
{
public:
  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  ~UBI0()
  {
  };
};  // end class

template<>
class UBI0<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>
{
public:
  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  ~UBI0()
  {
  };
};  // end class

template<>
class UBI0<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND> :
        public GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
                ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>
{
public:
  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  UBI0(ros::NodeHandle nh, std::string device_id, std::vector<generic_updater::UpdateConfig> update_configs_vector,
       operation_mode::device_update_state::DeviceUpdateState update_state,
       boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>(nh, device_id, update_configs_vector, update_state)
  {
    ROS_ERROR("This UBI0 tactile object should not have been instantiated for this type of ETHERCAT_DATA_STRUCTURE");
  }

  ~UBI0()
  {
  };
};  // end class

}  // namespace tactiles

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _UBI0_HPP_ */
