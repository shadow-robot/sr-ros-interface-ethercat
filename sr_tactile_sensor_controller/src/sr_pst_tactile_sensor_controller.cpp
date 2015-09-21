/**
 * @file   sr_pst_tactile_sensor_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief  Publishes PST tactile state
 *
 */

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_pst_tactile_sensor_controller.hpp"
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace controller
{
  bool SrPSTTactileSensorController::init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    bool ret=SrTactileSensorController::init(hw, root_nh, controller_nh);
    if (ret)
    {
      // realtime publisher
      pst_realtime_pub_ = PSTPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::ShadowPST>(nh_prefix_, "tactile", 4));
    }
    return ret;
  }

  void SrPSTTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
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
} //end namespace


PLUGINLIB_EXPORT_CLASS(controller::SrPSTTactileSensorController, controller_interface::ControllerBase)

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
