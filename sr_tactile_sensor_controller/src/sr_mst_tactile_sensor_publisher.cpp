/* Copyright 2023 Shadow Robot Company Ltd.
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
  * @file  sr_mst_tactile_sensor_publisher.cpp
  *
  * @brief Publish MST tactile state
  */

#include "sr_tactile_sensor_controller/sr_mst_tactile_sensor_publisher.hpp"

namespace controller
{
  /**
    * Initialize the MST sensor publisher.
    *
    * @param time The current process time with wich to initialize last_publish_time_
    */
  void SrMSTTactileSensorPublisher::init(const ros::Time& time)
  {
    // Tracks the last time the sensors were published (used to enforce the publish rate)
    last_publish_time_ = time;

    // Instantiate MST realtime publisher object
    mst_realtime_pub_ = MSTPublisherPtr(
      new realtime_tools::RealtimePublisher<sr_robot_msgs::MSTAll>(nh_prefix_, "tactile", 4));
  }

  /**
    * Populate and publish the MST sensor data.
    *
    * @param time The current process time.
    * @param period The time since the last call to update. Not used by this class
    */
  void SrMSTTactileSensorPublisher::update(const ros::Time& time, const ros::Duration& period)
  {
    // Limit data publishing rate according to value defined in publish_rate_
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    {
      // Publish MST tactile data
      if (mst_realtime_pub_->trylock())
      {
        // Update time in last_publish_time_
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
        // Populate message to be published
        mst_realtime_pub_->msg_ = sensors_->at(0).mst.sensor_data;  // contains data of sensors on all fingers
        mst_realtime_pub_->msg_.header.stamp = time;
        mst_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
        mst_realtime_pub_->unlockAndPublish();
      }
    }
  }
}  // end namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
