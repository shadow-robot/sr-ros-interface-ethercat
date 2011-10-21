/**
 * @file   generic_tactiles.cpp
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

#include "sr_robot_lib/generic_tactiles.hpp"

namespace tactiles
{
  const unsigned int GenericTactiles::nb_tactiles = 5;

  GenericTactiles::GenericTactiles(std::vector<generic_updater::UpdateConfig> update_configs_vector)
  {
    sensor_updater = boost::shared_ptr<generic_updater::SensorUpdater>(new generic_updater::SensorUpdater(update_configs_vector));
    reset_service_client_ = nodehandle_.advertiseService("/tactiles/reset", &GenericTactiles::reset, this);
  }

  /**
   * Reset the tactile sensors.
   *
   * @param request empty
   * @param response empty
   *
   * @return true if success
   */
  bool GenericTactiles::reset(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
  {
    ROS_INFO_STREAM("Resetting tactiles");
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


