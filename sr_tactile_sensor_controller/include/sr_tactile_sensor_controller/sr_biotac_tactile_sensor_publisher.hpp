/**
 * @file   sr_biotac_tactile_sensor_publisher.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes Biotac tactile state
 *
 */

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/BiotacAll.h>
#include <vector>
#include <string>

namespace controller
{

class SrBiotacTactileSensorPublisher: public SrTactileSensorPublisher
{
public:
  SrBiotacTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors,
                                 double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
    : SrTactileSensorPublisher(sensors, publish_rate, nh_prefix, prefix) {}
  virtual void init(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);

private:
  typedef realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll> BiotacPublisher;
  typedef boost::shared_ptr<BiotacPublisher > BiotacPublisherPtr;
  BiotacPublisherPtr biotac_realtime_pub_;
};

}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
