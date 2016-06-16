/**
 * @file   sr_biotac_tactile_sensor_publisher.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes Biotac tactile state
 *
 */

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_biotac_tactile_sensor_publisher.hpp"
#include <pluginlib/class_list_macros.h>


namespace controller
{
void SrBiotacTactileSensorPublisher::init(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;

  // realtime publisher
  biotac_realtime_pub_ = BiotacPublisherPtr(
    new realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll>(nh_prefix_, "tactile", 4));
  biotac_realtime_pub_->lock();
  for (unsigned i = 0; i < sensors_->size(); i++)
  {
    biotac_realtime_pub_->msg_.tactiles[i].electrodes.resize(sensors_->at(i).biotac.electrodes.size());
  }
  biotac_realtime_pub_->unlock();
}

void SrBiotacTactileSensorPublisher::update(const ros::Time& time, const ros::Duration& period)
{
  bool biotac_published = false;
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (biotac_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
      biotac_published = true;
      // populate message
      biotac_realtime_pub_->msg_.header.stamp = time;
      biotac_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
      // data
      for (unsigned i = 0; i < sensors_->size(); i++)
      {
        biotac_realtime_pub_->msg_.tactiles[i].pac0 = sensors_->at(i).biotac.pac0;
        biotac_realtime_pub_->msg_.tactiles[i].pac1 = sensors_->at(i).biotac.pac1;
        biotac_realtime_pub_->msg_.tactiles[i].pdc = sensors_->at(i).biotac.pdc;
        biotac_realtime_pub_->msg_.tactiles[i].tac = sensors_->at(i).biotac.tac;
        biotac_realtime_pub_->msg_.tactiles[i].tdc = sensors_->at(i).biotac.tdc;
        biotac_realtime_pub_->msg_.tactiles[i].electrodes = sensors_->at(i).biotac.electrodes;
      }
      biotac_realtime_pub_->unlockAndPublish();
    }
  }
}
}  // end namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
