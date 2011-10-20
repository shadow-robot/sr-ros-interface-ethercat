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
}

#include <ros/ros.h>
#include <vector>
#include <std_srvs/Empty.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include "sr_robot_lib/tactile_sensors.hpp"

namespace tactiles
{
  class GenericTactiles
  {
  public:
    GenericTactiles();
    ~GenericTactiles() {};

    virtual void update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data) = 0;

    virtual void build_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command) = 0;

    /**
     * Publish the information to a ROS topic.
     *
     */
    virtual void publish() = 0;

    /**
     * This function adds the diagnostics for the tactiles to the
     * multi diagnostic status published by the hand.
     */
    virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                 diagnostic_updater::DiagnosticStatusWrapper &d) = 0;

    /**
     * Reset the tactile sensors.
     *
     * @param request empty
     * @param response empty
     *
     * @return true if success
     */
    bool reset(std_srvs::Empty::Request& request,
               std_srvs::Empty::Response& response);

    /// Number of tactile sensors (TODO: should probably be defined in the protocol)
    static const unsigned int nb_tactiles;

  protected:
    ros::NodeHandle nodehandle_;

    ros::ServiceServer reset_service_client_;

  };//end class
}//end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* GENERIC_TACTILES_HPP_ */
