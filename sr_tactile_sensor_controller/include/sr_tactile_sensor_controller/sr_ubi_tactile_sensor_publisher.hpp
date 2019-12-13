/*
* @file   sr_ubi_tactile_sensor_publisher.hpp
* @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
* @date   Aug 25 2014
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
* @brief  Publishes ubi tactile state
*
*/

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/UBI0All.h>
#include <sr_robot_msgs/MidProxDataAll.h>
#include <vector>
#include <string>

namespace controller
{

class SrUbiTactileSensorPublisher: public SrTactileSensorPublisher
{
public:
  SrUbiTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors, double publish_rate,
                              ros::NodeHandle nh_prefix, std::string prefix)
        : SrTactileSensorPublisher(sensors, publish_rate, nh_prefix, prefix) {}
  virtual void init(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);

private:
  typedef realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All> UbiPublisher;
  typedef boost::shared_ptr<UbiPublisher > UbiPublisherPtr;
  typedef realtime_tools::RealtimePublisher<sr_robot_msgs::MidProxDataAll> MidProxPublisher;
  typedef boost::shared_ptr<MidProxPublisher > MidProxPublisherPtr;
  UbiPublisherPtr ubi_realtime_pub_;
  MidProxPublisherPtr midprox_realtime_pub_;
};

}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

