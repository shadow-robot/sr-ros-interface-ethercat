/**
* @file   sr_tactile_sensor_publisher.hpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
*
* Copyright 2015 Shadow Robot Company Ltd.
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
* @brief  Publishes PST tactile state
*
*/

#include "sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp"
#include <vector>
#include <string>

namespace controller
{
SrTactileSensorPublisher::SrTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors,
                                                   double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
{
  sensors_ = sensors;
  publish_rate_ = publish_rate;
  nh_prefix_ = nh_prefix;
  prefix_ = prefix;
}
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
