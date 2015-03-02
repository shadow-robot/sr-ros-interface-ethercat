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
/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_tactile_sensor_controller.hpp"

using namespace std;

namespace controller
{
  bool SrTactileSensorController::init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    // get all sensors from the hardware interface
    // apparently all the actuators have the tactile data copied in, so take the first one.
    sr_actuator::SrMotorActuator* motor_actuator=NULL;
    if (hw->transmissions_.size()>0)
    {
      motor_actuator = static_cast<sr_actuator::SrMotorActuator*>  (hw->transmissions_[0].actuator_);
    }
    //sr_actuator::SrMotorActuator* motor_actuator = static_cast<sr_actuator::SrMotorActuator*> (hw->getActuator("FFJ0"));
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
      ROS_ERROR("Could not find at least an actuator with tactile data inside");
      return false;
    }
  }
  
  void SrTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
  {
  }

  void SrTactileSensorController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }
  
   void SrTactileSensorController::stopping(const ros::Time& time)
  {}
}




/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


