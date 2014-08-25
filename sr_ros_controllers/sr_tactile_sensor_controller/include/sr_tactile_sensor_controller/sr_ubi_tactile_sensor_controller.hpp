/**
 * @file   sr_ubi_tactile_sensor_controller.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Aug 25 2014
 *
 *
 * @brief  Publishes ubi tactile state
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

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#ifndef SR_UBI_TACTILE_SENSOR_CONTROLLER_H
#define SR_UBI_TACTILE_SENSOR_CONTROLLER_H

#include <sr_tactile_sensor_controller/sr_tactile_sensor_controller.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/UBI0All.h>
#include <sr_robot_msgs/MidProxDataAll.h>
#include <pluginlib/class_list_macros.h>

namespace controller
{

  class SrUbiTactileSensorController: public SrTactileSensorController
  {
  public:
    SrUbiTactileSensorController() : SrTactileSensorController() {}
    virtual bool init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
    virtual void update(const ros::Time& time, const ros::Duration& period);

  private:
   
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::UBI0All> > UbiPublisherPtr;
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::MidProxDataAll> > MidProxPublisherPtr;
    UbiPublisherPtr ubi_realtime_pub_;
    MidProxPublisherPtr midprox_realtime_pub_;

  };

}// namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* SR_UBI_TACTILE_SENSOR_CONTROLLER_H */
