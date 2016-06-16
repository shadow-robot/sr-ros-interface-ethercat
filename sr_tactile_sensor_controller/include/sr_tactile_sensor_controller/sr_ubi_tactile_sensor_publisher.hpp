/**
 * @file   sr_ubi_tactile_sensor_publisher.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Aug 25 2014
 *
 * Copyright 2015 Shadow Robot Company Ltd.
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

