/*
* @file   sr_pst_tactile_sensor_publisher.hpp
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

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/ShadowPST.h>
#include <vector>
#include <string>

namespace controller
{

class SrPSTTactileSensorPublisher: public SrTactileSensorPublisher
{
public:
  SrPSTTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors,
                              double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
    : SrTactileSensorPublisher(sensors, publish_rate, nh_prefix, prefix) {}
  virtual void init(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);

private:
  typedef realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST> PSTPublisher;
  typedef boost::shared_ptr<PSTPublisher > PSTPublisherPtr;
  PSTPublisherPtr pst_realtime_pub_;
};

}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
