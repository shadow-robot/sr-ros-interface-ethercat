/**
 * @file   generic_tactiles.hpp
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
 * @brief This is the main class for accessing the data from the
 *        tactiles.
 *
 *
 */

#ifndef GENERIC_TACTILES_HPP_
#define GENERIC_TACTILES_HPP_

#include <boost/smart_ptr.hpp>
#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
#include <sr_external_dependencies/external/0230_palm_edc_TS/0230_palm_edc_ethercat_protocol.h>
#include <sr_external_dependencies/external/0240_palm_edc_IMU/0240_palm_edc_IMU_ethercat_protocol.h>
#include <sr_external_dependencies/external/0320_palm_edc_muscle/0320_palm_edc_ethercat_protocol.h>
}

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_srvs/Empty.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <sr_hardware_interface/tactile_sensors.hpp>
#include "sr_robot_lib/generic_updater.hpp"
#include "sr_robot_lib/sensor_updater.hpp"

namespace tactiles
{
template<class StatusType, class CommandType>
class GenericTactiles
{
public:
  GenericTactiles(ros::NodeHandle nh, std::string device_id,
                  std::vector<generic_updater::UpdateConfig> update_configs_vector,
                  operation_mode::device_update_state::DeviceUpdateState update_state);

  virtual ~GenericTactiles()
  {
  }

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

  /**
   * Reset the tactile sensors.
   *
   * @param request empty
   * @param response empty
   *
   * @return true if success
   */
  bool reset(std_srvs::Empty::Request &request,
             std_srvs::Empty::Response &response);

  /// Number of tactile sensors (TODO: should probably be defined in the protocol)
  static const unsigned int nb_tactiles;

  boost::shared_ptr<generic_updater::SensorUpdater<CommandType> > sensor_updater;
  /// the vector containing the data for the tactiles.
  boost::shared_ptr<std::vector<GenericTactileData> > tactiles_vector;

  virtual std::vector<AllTactileData> *get_tactile_data();

protected:
  void process_received_data_type(int32u data);

  ros::NodeHandle nodehandle_;
  std::string device_id_;

  ros::ServiceServer reset_service_client_;

  // Contains the received data types.
  std::vector<int32u> initialization_received_data_vector;

  /**
   * Sanitise a string coming from the palm. Make sure we're not
   * outputting garbage in the diagnostics topic.
   * The acceptable range for the char is [0x20 .. 0x7E]
   *
   * @param raw_string The incoming raw string
   * @param str_size The max size of the string
   *
   * @return The sanitised string.
   */
  std::string sanitise_string(const char *raw_string, const unsigned int str_size);

  boost::shared_ptr<std::vector<AllTactileData> > all_tactile_data;
};  // end class
}  // namespace tactiles

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* GENERIC_TACTILES_HPP_ */
