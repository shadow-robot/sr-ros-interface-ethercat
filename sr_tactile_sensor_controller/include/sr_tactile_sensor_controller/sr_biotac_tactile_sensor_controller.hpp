/**
 * @file   sr_biotac_tactile_sensor_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief  Publishes Biotac tactile state
 *
 */

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_controller.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_robot_msgs/BiotacAll.h>

namespace controller
{

  class SrBiotacTactileSensorController: public SrTactileSensorController
  {
  public:
    SrBiotacTactileSensorController() : SrTactileSensorController() {}
    virtual bool init(ros_ethercat_model::RobotState* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
    virtual void update(const ros::Time& time, const ros::Duration& period);

  private:

    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::BiotacAll> > BiotacPublisherPtr;
    BiotacPublisherPtr biotac_realtime_pub_;
  };

}// namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
