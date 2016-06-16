/**
 * @file   sr_tactile_sensor_publisher.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes PST tactile state
 *
 */

#include "sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp"
#include <vector>
#include <string>

namespace controller
{
SrTactileSensorPublisher::SrTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors,
                                                   double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
{
  sensors_ = sensors;
  publish_rate_ = publish_rate;
  nh_prefix_ = nh_prefix;
  prefix_ = prefix;
}
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
