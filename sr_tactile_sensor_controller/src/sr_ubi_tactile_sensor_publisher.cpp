/**
* @file   sr_ubi_tactile_sensor_publisher.cpp
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
* @brief  Publishes ubi tactile state.
*
*/

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_ubi_tactile_sensor_publisher.hpp"
#include <pluginlib/class_list_macros.h>

namespace controller
{
void SrUbiTactileSensorPublisher::init(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;

  // realtime publisher
  ubi_realtime_pub_ = UbiPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All>(
    nh_prefix_, "tactile", 4));
  midprox_realtime_pub_ = MidProxPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::MidProxDataAll>(
    nh_prefix_, "tactile_mid_prox", 4));
}

void SrUbiTactileSensorPublisher::update(const ros::Time& time, const ros::Duration& period)
{
  bool ubi_published = false;
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (ubi_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      ubi_published = true;
      // populate message
      ubi_realtime_pub_->msg_.header.stamp = time;
      ubi_realtime_pub_->msg_.header.frame_id = prefix_ + "distal";
      // data
      for (unsigned i = 0; i < sensors_->size(); i++)
      {
        sr_robot_msgs::UBI0 tactile_tmp;

        tactile_tmp.distal = sensors_->at(i).ubi0.distal;
        ubi_realtime_pub_->msg_.tactiles[i] = tactile_tmp;
      }
      ubi_realtime_pub_->unlockAndPublish();
    }

     // try to publish
    if (midprox_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      if (!ubi_published)
      {
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      }
      // populate message
      midprox_realtime_pub_->msg_.header.stamp = time;
      midprox_realtime_pub_->msg_.header.frame_id = prefix_ + "proximal";
      // data
      for (unsigned i = 0; i < sensors_->size(); i++)
      {
        sr_robot_msgs::MidProxData midprox_tmp;

        midprox_tmp.middle = sensors_->at(i).ubi0.middle;
        midprox_tmp.proximal = sensors_->at(i).ubi0.proximal;
        midprox_realtime_pub_->msg_.sensors[i] = midprox_tmp;
      }
      midprox_realtime_pub_->unlockAndPublish();
    }
  }
}
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
