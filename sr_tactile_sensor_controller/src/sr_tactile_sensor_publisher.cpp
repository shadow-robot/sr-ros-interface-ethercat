/**
 * @file   sr_pst_tactile_sensor_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief  Publishes PST tactile state
 *
 */

/// Original author of ImuSensorController : Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp"

using namespace std;

namespace controller
{
  SrTactileSensorPublisher::SrTactileSensorPublisher(std::vector<tactiles::AllTactileData>* sensors, double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
  {
    sensors_ = sensors;
    publish_rate_ = publish_rate;
    nh_prefix_ = nh_prefix;
    prefix_ = prefix;
  }
} //end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
