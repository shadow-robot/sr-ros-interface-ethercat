 /**
 * @file   sr_tactile_sensor_controller.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Aug 22 2014
 *
 * Copyright 2014 University of Bielefeld
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
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
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

#pragma once

#include <controller_interface/controller.h>
#include <sr_robot_lib/generic_tactiles.hpp>
#include <sr_hardware_interface/sr_actuator.hpp>
#include <boost/shared_ptr.hpp>
#include <ros_ethercat_model/robot_state_interface.hpp>

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <vector>
#include <string>

namespace controller
{
// this controller gets access to the SrTactileSensorInterface
class SrTactileSensorController: public controller_interface::Controller<ros_ethercat_model::RobotStateInterface>
{
public:
  SrTactileSensorController();
  virtual bool init(ros_ethercat_model::RobotStateInterface* hw,
                    ros::NodeHandle &root_nh,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);
  virtual void stopping(const ros::Time& time);

protected:
  std::vector<tactiles::AllTactileData>* sensors_;
  double publish_rate_;
  ros::NodeHandle nh_prefix_;
  std::string prefix_;
  bool initialized_;
  boost::shared_ptr<SrTactileSensorPublisher> sensor_publisher_;
};

}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

