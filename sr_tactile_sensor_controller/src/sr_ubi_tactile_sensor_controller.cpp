/**
 * @file   sr_ubi_tactile_sensor_controller.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Aug 25 2014
 *
 *
 * @brief  Publishes ubi tactile state.
 *
 */

///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_ubi_tactile_sensor_controller.hpp"
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace controller
{
  bool SrUbiTactileSensorController::init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    bool ret=SrTactileSensorController::init(hw, root_nh, controller_nh);
    if (ret)
    {
      // realtime publisher
      ubi_realtime_pub_ = UbiPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All>(root_nh, "tactile", 4));
      midprox_realtime_pub_ = MidProxPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::MidProxDataAll>(root_nh, "tactile_mid_prox", 4));
    }
    return ret;
  }

  void SrUbiTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
  {
    
    using namespace hardware_interface;
    bool ubi_published=false;
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){
      // try to publish
      if (ubi_realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
        ubi_published=true;
        // populate message
        ubi_realtime_pub_->msg_.header.stamp = time;
        ubi_realtime_pub_->msg_.header.frame_id = "palm";
        // data
        for (unsigned i=0; i<sensors_->size(); i++){
          sr_robot_msgs::UBI0 tactile_tmp;

          tactile_tmp.distal = sensors_->at(i).ubi0.distal;
          ubi_realtime_pub_->msg_.tactiles[i] = tactile_tmp;
        }
        ubi_realtime_pub_->unlockAndPublish();

      }

       // try to publish
      if (midprox_realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        if( !ubi_published)
          last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
        // populate message
        midprox_realtime_pub_->msg_.header.stamp = time;
        midprox_realtime_pub_->msg_.header.frame_id = "palm";
        // data
        for (unsigned i=0; i<sensors_->size(); i++){
          sr_robot_msgs::MidProxData midprox_tmp;

          midprox_tmp.middle = sensors_->at(i).ubi0.middle;
          midprox_tmp.proximal = sensors_->at(i).ubi0.proximal;
          midprox_realtime_pub_->msg_.sensors[i] = midprox_tmp;
        }
        midprox_realtime_pub_->unlockAndPublish();
      }
    }
  }
}


PLUGINLIB_EXPORT_CLASS(controller::SrUbiTactileSensorController, controller_interface::ControllerBase)

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
