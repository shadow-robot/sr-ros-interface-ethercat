/**
 * @file   sr_motor_hand_lib.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  3 12:12:13 2011
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief This is a library for the etherCAT hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#ifndef _SR_MOTOR_HAND_LIB_HPP_
#define _SR_MOTOR_HAND_LIB_HPP_

#include "sr_robot_lib/sr_motor_robot_lib.hpp"
#include <std_srvs/Empty.h>

// to be able to load the configuration from the
// parameter server
#include <ros/ros.h>
#include <string>
#include <utility>
#include <map>
#include <vector>

namespace shadow_robot
{
template<class StatusType, class CommandType>
class SrMotorHandLib :
        public SrMotorRobotLib<StatusType, CommandType>
{
public:
  SrMotorHandLib(hardware_interface::HardwareInterface *hw, ros::NodeHandle nh, ros::NodeHandle nhtilde,
                 std::string device_id, std::string joint_prefix);

  /**
   * The service callback for setting the Force PID values. There's only one callback
   * function, but it can called for any motors. We know which motor called the service
   * thanks to the motor_index.
   *
   * @param request The request contains the new parameters for the controllers.
   * @param response True if succeeded.
   * @param motor_index The index of the motor for which the service has been called.
   *
   * @return true if succeeded.
   */
  bool force_pid_callback(sr_robot_msgs::ForceController::Request &request,
                          sr_robot_msgs::ForceController::Response &response,
                          int motor_index);

  /**
   * Reset the motor at motor index.
   *
   * @param request empty
   * @param response empty
   * @param joint A pair containing the index of the motor for the given
   *              joint followed by the name of the joint we're resetting
   *
   * @return true if success
   */
  bool reset_motor_callback(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response,
                            std::pair<int, std::string> joint);

#ifdef DEBUG_PUBLISHER
  /**
   * This is a service callback: we set the debug data we want to publish
   * at full speed in the debug topics.
   *
   * @param request Contains the motor index and the MOTOR_DATA type
   * @param response True if succeeded.
   *
   * @return true if succeeded.
   */
  bool set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                 sr_robot_msgs::SetDebugData::Response& response);
#endif

protected:
  /**
   * Initializes the hand library with the needed values.
   *
   * @param joint_names A vector containing all the joint names.
   * @param actuator_ids A vector containing the corresponding actuator ids.
   * @param joint_to_sensors A vector mapping the joint to the sensor index we read from the palm.
   */
  virtual void initialize(std::vector<std::string> joint_names, std::vector<int> actuator_ids,
                          std::vector<shadow_joints::JointToSensor> joint_to_sensors);

  /**
   * Updates the parameter values for the force control in the Parameter Server
   *
   * @param joint_name The name of the joint.
   * @param max_pwm The max pwm the motor will apply
   * @param sg_left Strain gauge left
   * @param sg_right Strain gauge right
   * @param f The feedforward term (directly adds f*error to the output of the PID)
   * @param p The p value.
   * @param i the i value.
   * @param d the d value.
   * @param imax the imax value.
   * @param deadband the deadband on the force.
   * @param sign can be 0 or 1 depending on the way the motor is plugged in.
   */
  void update_force_control_in_param_server(
    std::string joint_name, int max_pwm, int sg_left, int sg_right, int f, int p, int i,
    int d, int imax, int deadband, int sign, int torque_limit, int torque_limiter_gain);

  /**
   * Finds the joint name for a certain motor index
   *
   * @param motor_index The integer motor index
   */
  std::string find_joint_name(int motor_index);

private:
  /**
   * Reads the mapping associating a joint to a motor.
   * If the motor index is -1, then no motor is associated
   * to this joint.
   *
   *
   * @return a vector of motor indexes, ordered by joint.
   */
  std::vector<int> read_joint_to_motor_mapping();


  static const int nb_motor_data;
  static const char *human_readable_motor_data_types[];
  static const int32u motor_data_types[];

  /**
   * Read the motor board force pids from the parameter servers,
   * called when resetting the motor.
   *
   * @param joint_name the joint we want to reset
   * @param motor_index the index of the motor for this joint
   */
  void resend_pids(std::string joint_name, int motor_index);

  /**
   * A map used to keep the timers created in reset_motor_callback alive.
   * We're using a map to keep only one timer per joint.
   */
  std::map<std::string, ros::Timer> pid_timers;
};
}  // namespace shadow_robot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
