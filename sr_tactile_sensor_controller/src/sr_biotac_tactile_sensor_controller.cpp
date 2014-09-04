/**
 * @file   sr_biotac_tactile_sensor_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief  Publishes Biotac tactile state
 *
 */

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_biotac_tactile_sensor_controller.hpp"
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace controller
{
  bool SrBiotacTactileSensorController::init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    bool ret=SrTactileSensorController::init(hw, root_nh, controller_nh);
    if (ret)
    {
      // realtime publisher
      biotac_realtime_pub_ = BiotacPublisherPtr(new realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll>(root_nh, "biotac_tactile", 4));
    }
    return ret;
  }

  void SrBiotacTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
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
        biotac_realtime_pub_->msg_.header.frame_id = "palm";
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
} //end namespace


PLUGINLIB_EXPORT_CLASS(controller::SrBiotacTactileSensorController, controller_interface::ControllerBase)

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
