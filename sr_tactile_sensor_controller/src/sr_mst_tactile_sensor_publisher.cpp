/* 
*Copyright 2023 Shadow Robot Company Ltd.
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
  * @file   sr_mst_tactile_sensor_publisher.cpp
  * @author Rodrigo Zenha <rodrigo@shadowrobot.com>
  *
  * @brief Publish MST tactile state
  */

#include "sr_tactile_sensor_controller/sr_mst_tactile_sensor_publisher.hpp"

namespace controller
{
void SrMSTTactileSensorPublisher::init(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;

  // realtime publisher
  mst_realtime_pub_ = MSTPublisherPtr(
    new realtime_tools::RealtimePublisher<sr_robot_msgs::MSTAll>(nh_prefix_, "tactile", 4));
}

void SrMSTTactileSensorPublisher::update(const ros::Time& time, const ros::Duration& period)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (mst_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      // populate message
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
