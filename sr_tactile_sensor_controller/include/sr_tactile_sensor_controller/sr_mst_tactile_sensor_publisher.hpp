/* Copyright 2023-2024 Shadow Robot Company Ltd.
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
  * @file  sr_mst_tactile_sensor_publisher.hpp
  *
  * @brief Publish MST tactile state
  */


#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/MSTAll.h>
#include <vector>
#include <string>

namespace controller
{
/** 
* Class that contains all functions and member variables relevant to publish the MST sensor data.
* It inherits from the SrTactileSensorPublisher class.
*/ 
class SrMSTTactileSensorPublisher: public SrTactileSensorPublisher
{
  public:
    /**
      * MST tactile sensor publisher constructor.
      *
      * @param sensors Is a pointer to the current tactile data to be published
      * @param publish_rate Defines the data publishing time rate.
      * @param nh_prefix The prefix of the ROS node handle to be used for publishing.
      * @param prefix The hand prefix (e.g. "rh_" or "lh_")
      */
    SrMSTTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors,
                                double publish_rate, ros::NodeHandle nh_prefix, std::string prefix) :
      SrTactileSensorPublisher(sensors, publish_rate, nh_prefix, prefix)
      {
      }

    /**
      * Initialize the MST sensor publisher.
      *
      * @param time The current process time with wich to initialize last_publish_time_
      */
    virtual void init(const ros::Time& time);

    /**
      * Populate and publish the MST sensor data.
      *
      * @param time The current process time.
      * @param period The time since the last call to update. Not used by this class
      */
    virtual void update(const ros::Time& time, const ros::Duration& period);

  private:
    // ROS realtime publisher of sr_robot_msgs::MSTAll
    typedef realtime_tools::RealtimePublisher<sr_robot_msgs::MSTAll> MSTPublisher;
    typedef boost::shared_ptr<MSTPublisher > MSTPublisherPtr;
    MSTPublisherPtr mst_realtime_pub_;
};
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
