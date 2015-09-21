/**
 * @file   sr_tactile_sensor_controller.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Aug 22 2014
 *
 *
 * @brief  Generic controller for tactile sensor data publishing.
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
/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_tactile_sensor_controller.hpp"
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace controller
{
  SrTactileSensorController::SrTactileSensorController()
      : initialized_(false)
  {}

  bool SrTactileSensorController::init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    
    if (!controller_nh.getParam("prefix", prefix_)){
      ROS_ERROR("Parameter 'prefix' not set");
      return false;
    }
    
    //this should handle the case where we don't want a prefix
    if (!prefix_.empty())
    {
      nh_prefix_ = ros::NodeHandle(root_nh, prefix_);
      prefix_+="_";
    }
    else
    {
      nh_prefix_ = ros::NodeHandle(root_nh);
    }
    
    // get all sensors from the hardware interface
    // apparently all the actuators have the tactile data copied in, so take the first one.
    sr_actuator::SrMotorActuator* motor_actuator = static_cast<sr_actuator::SrMotorActuator*> (hw->getActuator(prefix_+"FFJ0"));
    if (motor_actuator)
    {
      sensors_ = motor_actuator->motor_state_.tactiles_;

      // get publishing period
      if (!controller_nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
      }
         
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Could not find the "<<prefix_<<"FFJ0 actuator");
      return false;
    }
  }
  
  void SrTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
  {
    if (!initialized_)
    {
      if (!sensors_->empty())
      {
        if (!sensors_->at(0).type.empty())
        {
          if (sensors_->at(0).type == "pst")
          {
            pst_init();
          }
          else if (sensors_->at(0).type == "biotac")
          {
            biotac_init();
          }
          else if (sensors_->at(0).type == "ubi")
          {
            ubi_init();
          }
          initialized_ = true;
        }
      }
    }
    else
    {
      if (sensors_->at(0).type == "pst")
      {
        pst_update(time, period);
      }
      else if (sensors_->at(0).type == "biotac")
      {
        biotac_update(time, period);
      }
      else if (sensors_->at(0).type == "ubi")
      {
        ubi_update(time, period);
      }
    }
  }

  void SrTactileSensorController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }
  
  void SrTactileSensorController::stopping(const ros::Time& time)
  {}

  void SrTactileSensorController::pst_init()
  {
    // realtime publisher
    pst_realtime_pub_ = PSTPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST>(nh_prefix_, "tactile", 4));
  }

  void SrTactileSensorController::pst_update(const ros::Time& time, const ros::Duration& period)
  {
    using namespace hardware_interface;
    bool pst_published=false;
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
    {
      // try to publish
      if (pst_realtime_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
        pst_published=true;
        // populate message
        pst_realtime_pub_->msg_.header.stamp = time;
        pst_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
        // data
        for (unsigned i=0; i<sensors_->size(); i++)
        {
          pst_realtime_pub_->msg_.pressure[i] = sensors_->at(i).pst.pressure;
          pst_realtime_pub_->msg_.temperature[i] = sensors_->at(i).pst.temperature;
        }
        pst_realtime_pub_->unlockAndPublish();
      }
    }
  }

  void SrTactileSensorController::biotac_init()
  {
    // realtime publisher
    biotac_realtime_pub_ = BiotacPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll>(nh_prefix_, "tactile", 4));
  }

  void SrTactileSensorController::biotac_update(const ros::Time& time, const ros::Duration& period)
  {
    using namespace hardware_interface;
    bool biotac_published=false;
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
    {
      // try to publish
      if (biotac_realtime_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
        biotac_published=true;
        // populate message
        biotac_realtime_pub_->msg_.header.stamp = time;
        biotac_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
        // data
        for (unsigned i=0; i<sensors_->size(); i++)
        {
          biotac_realtime_pub_->msg_.tactiles[i].pac0 = sensors_->at(i).biotac.pac0;
          biotac_realtime_pub_->msg_.tactiles[i].pac1 = sensors_->at(i).biotac.pac1;
          biotac_realtime_pub_->msg_.tactiles[i].pdc = sensors_->at(i).biotac.pdc;
          biotac_realtime_pub_->msg_.tactiles[i].tac = sensors_->at(i).biotac.tac;
          biotac_realtime_pub_->msg_.tactiles[i].tdc = sensors_->at(i).biotac.tdc;

          for(size_t j=0 ; j < sensors_->at(i).biotac.electrodes.size() ; ++j)
            biotac_realtime_pub_->msg_.tactiles[i].electrodes[j] = sensors_->at(i).biotac.electrodes[j];
        }
        biotac_realtime_pub_->unlockAndPublish();
      }
    }
  }

  void SrTactileSensorController::ubi_init()
  {
    // realtime publisher
    ubi_realtime_pub_ = UbiPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All>(nh_prefix_, "tactile", 4));
    midprox_realtime_pub_ = MidProxPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::MidProxDataAll>(nh_prefix_, "tactile_mid_prox", 4));
  }

  void SrTactileSensorController::ubi_update(const ros::Time& time, const ros::Duration& period)
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
        ubi_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
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
        midprox_realtime_pub_->msg_.header.frame_id = prefix_+"proximal";
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


PLUGINLIB_EXPORT_CLASS(controller::SrTactileSensorController, controller_interface::ControllerBase)

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


