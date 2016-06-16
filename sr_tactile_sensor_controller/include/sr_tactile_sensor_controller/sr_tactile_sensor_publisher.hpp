/**
 * @file   sr_tactile_sensor_publisher.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes PST tactile state
 *
 */

#pragma once

#include <sr_hardware_interface/tactile_sensors.hpp>
#include <sr_robot_msgs/ShadowPST.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace controller
{

class SrTactileSensorPublisher
{
public:
  SrTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors, double publish_rate,
                           ros::NodeHandle nh_prefix, std::string prefix);
  virtual ~SrTactileSensorPublisher() {}
  virtual void init(const ros::Time& time) {}
  virtual void update(const ros::Time& time, const ros::Duration& period) {}

protected:
  std::vector<tactiles::AllTactileData>* sensors_;
  ros::Time last_publish_time_;
  double publish_rate_;
  ros::NodeHandle nh_prefix_;
  std::string prefix_;
};

}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
