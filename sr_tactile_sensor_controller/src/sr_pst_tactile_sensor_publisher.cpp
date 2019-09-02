/**
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

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_pst_tactile_sensor_publisher.hpp"

namespace controller
{
void SrPSTTactileSensorPublisher::init(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;

  // realtime publisher
  pst_realtime_pub_ = PSTPublisherPtr(
    new realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST>(nh_prefix_, "tactile", 4));
  pst_realtime_pub_->msg_.pressure.resize(sensors_->size());
  pst_realtime_pub_->msg_.temperature.resize(sensors_->size());
}

void SrPSTTactileSensorPublisher::update(const ros::Time& time, const ros::Duration& period)
{
  bool pst_published = false;
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (pst_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      pst_published = true;
      // populate message
      pst_realtime_pub_->msg_.header.stamp = time;
      pst_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
      // data
      for (unsigned i = 0; i < sensors_->size(); i++)
      {
        pst_realtime_pub_->msg_.pressure[i] = sensors_->at(i).pst.pressure;
        pst_realtime_pub_->msg_.temperature[i] = sensors_->at(i).pst.temperature;
      }
      pst_realtime_pub_->unlockAndPublish();
    }
  }
}
}  // end namespace controller


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
