/**
 * @file   sr_motor_robot_lib.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
 * @date   Tue Mar  19 17:12:13 2013
 *
 *
 * Copyright 2013 Shadow Robot Company Ltd.
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
 * @brief This is a generic robot library for Shadow Robot's motor-actuated Hardware.
 *
 *
 */

#ifndef _SR_MOTOR_ROBOT_LIB_HPP_
#define _SR_MOTOR_ROBOT_LIB_HPP_

#include "sr_robot_lib/sr_robot_lib.hpp"

#include <sr_robot_msgs/ForceController.h>
#include <sr_robot_msgs/ControlType.h>
#include <sr_robot_msgs/ChangeControlType.h>
#include <sr_robot_msgs/MotorSystemControls.h>
#include <sr_robot_msgs/ChangeMotorSystemControls.h>

#include "sr_robot_lib/motor_updater.hpp"
#include "sr_robot_lib/motor_data_checker.hpp"

#include <string>
#include <queue>
#include <utility>
#include <list>
#include <vector>

namespace shadow_robot
{
template<class StatusType, class CommandType>
class SrMotorRobotLib :
        public SrRobotLib<StatusType, CommandType>
{
public:
  SrMotorRobotLib(hardware_interface::HardwareInterface *hw, ros::NodeHandle nh, ros::NodeHandle nhtilde,
                  std::string device_id, std::string joint_prefix);

  /**
   * This function is called each time a new etherCAT message
   * is received in the sr06.cpp driver. It updates the joints_vector,
   * updating the different values, computing the calibrated joint
   * positions, etc... It also updates the tactile sensors values.
   *
   * @param status_data the received etherCAT message
   */
  void update(StatusType *status_data);

  /**
   * Builds a motor command: either send a torque demand or a configuration
   * demand if one is waiting.
   *
   * @param command The command we're building.
   */
  void build_command(CommandType *command);

  /**
   * This function adds the diagnostics for the hand to the
   * multi diagnostic status published in sr06.cpp.
   */
  void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                       diagnostic_updater::DiagnosticStatusWrapper &d);

  /**
   * Initiates the process to retrieve the initialization information from the motors
   */
  void reinitialize_motors();


  // Current update state of the motor (initialization, operation..)
  operation_mode::device_update_state::DeviceUpdateState motor_current_state;


protected:
  /**
   * Initializes the hand library with the needed values.
   *
   * @param joint_names A vector containing all the joint names.
   * @param actuator_ids A vector containing the corresponding actuator ids.
   * @param joint_to_sensors A vector mapping the joint to the sensor index we read from the palm.
   */
  virtual void initialize(std::vector<std::string> joint_names, std::vector<int> actuator_ids,
                          std::vector<shadow_joints::JointToSensor> joint_to_sensors) = 0;

  /**
   * Compute the calibrated position for the given joint. This method is called
   * from the update method, each time a new message is received.
   *
   * @param joint_tmp The joint we want to calibrate.
   * @param status_data The status information that comes from the robot
   */
  void calibrate_joint(std::vector<shadow_joints::Joint>::iterator joint_tmp, StatusType *status_data);

  /**
   * Read additional data from the latest message and stores it into the
   * joints_vector.
   *
   * @param joint_tmp The joint we want to read the data for.
   * @param status_data The status information that comes from the robot
   */
  void read_additional_data(std::vector<shadow_joints::Joint>::iterator joint_tmp, StatusType *status_data);

  /**
   * Calibrates and filters the position information (and computes velocity) for a give joint.
   * This method is called from the update method, each time a new message is received.
   *
   * @param joint_tmp The joint we process data from.
   * @param status_data The status information that comes from the robot
   * @param timestamp Timestamp of the data acquisition time
   */
  void process_position_sensor_data(std::vector<shadow_joints::Joint>::iterator joint_tmp, StatusType *status_data,
                                    double timestamp);

  /**
   * Transforms the incoming flag as a human
   * readable vector of strings.
   *
   * @param flag incoming flag.
   *
   * @return human readable flags.
   */
  std::vector<std::pair<std::string, bool> > humanize_flags(int flag);

  /**
   * Generates a force control config and adds it to the reconfig_queue with its
   * CRC. The config will be sent as soon as possible.
   *
   * @param motor_index The motor index.
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
  void generate_force_control_config(
    int motor_index, int max_pwm, int sg_left, int sg_right, int f, int p, int i, int d,
    int imax, int deadband, int sign, int torque_limit, int torque_limiter_gain);

  /**
   * Returns a pointer to the actuator for a certain joint.
   *
   * @param joint_tmp The joint we want to get the actuator from.
   *
   * @return a pointer to the actuator
   */
  sr_actuator::SrMotorActuator *get_joint_actuator(std::vector<shadow_joints::Joint>::iterator joint_tmp)
  {
    return static_cast<sr_actuator::SrMotorActuator *>(joint_tmp->actuator_wrapper->actuator);
  }

  /**
   * The motor updater is used to create a correct command to send to the motor.
   * It's build_command() is called each time the SR06::packCommand()
   * is called.
   */
  boost::shared_ptr<generic_updater::MotorUpdater<CommandType> > motor_updater_;


  /**
   * The ForceConfig type consists of an int representing the motor index for this config
   * followed by a vector of config: the index in the vector of config corresponds to the
   * type of the data, and the value at this index corresponds to the value we want to set.
   */
  typedef std::pair<int, std::vector<crc_unions::union16> > ForceConfig;
  /**
   * This queue contains the force PID config waiting to be pushed to the motor.
   */
  std::queue<ForceConfig, std::list<ForceConfig> > reconfig_queue;
  // this index is used to iterate over the config we're sending.
  int config_index;

  // contains a queue of motor indexes to reset
  std::queue<int16_t, std::list<int16_t> > reset_motors_queue;


  // The index of the motor in all the 20 motors
  int motor_index_full;
  // The index of the motor in the current message (from 0 to 9)
  int index_motor_in_msg;

  int8u crc_byte;
  int16u crc_result;
  int8u crc_i;


  // The update rate for each motor information
  std::vector<generic_updater::UpdateConfig> motor_update_rate_configs_vector;

  boost::shared_ptr<generic_updater::MotorDataChecker> motor_data_checker;


  // The current type of control (FORCE demand or PWM demand sent to the motors)
  sr_robot_msgs::ControlType control_type_;
  /**
   * Flag to signal that there has been a change in the value of control_type_ and certain actions are required.
   * The flag is set in the callback function of the change_control_type_ service.
   * The flag is checked in build_command() and the necessary actions are taken there.
   * These actions involve calling services in the controller manager and all the active controllers. This is the
   * reason why we don't do it directly in the callback function. As we use a single thread to serve the callbacks,
   * doing so would cause a deadlock, thus we do it in the realtime loop thread instead.
   */
  bool control_type_changed_flag_;
  // A service server used to change the control type on the fly.
  ros::ServiceServer change_control_type_;
  // A mutual exclusion object to ensure that no command will be sent to the robot while a change
  // in the control type (PWM or torque) is ongoing
  boost::shared_ptr<boost::mutex> lock_command_sending_;

  /**
   * The callback to the change_control_type_ service. Updates
   *  the current control_type_ with the requested control_type.
   *
   * @param request Requested control_type_
   * @param response The new control_type we'll use
   *
   * @return true if success, false if bad control type requested
   */
  bool change_control_type_callback_(sr_robot_msgs::ChangeControlType::Request &request,
                                     sr_robot_msgs::ChangeControlType::Response &response);

  /**
   * Load the necessary parameters in the Parameter Server and
   * calls a service for every controller currently loaded in the controller manager to make it
   * reload (resetGains()) its parameters from the Parameter Server
   *
   * @param control_type The new active control type (PWM or torque)
   *
   * @return true if all the steps successful
   */
  bool change_control_parameters(int16_t control_type);

  // The Flag which will be sent to change the motor controls
  std::queue<std::vector<sr_robot_msgs::MotorSystemControls>,
          std::list<std::vector<sr_robot_msgs::MotorSystemControls> > > motor_system_control_flags_;
  // A service server used to call the different motor system controls "buttons"
  ros::ServiceServer motor_system_control_server_;

  /**
   * The callback to the control_motor_ service. Sets the correct flags to 1 or 0
   *  for the MOTOR_SYSTEM_CONTROLS, to control the motors (backlash compensation
   *  on/off, increase sg tracking, jiggling, write config to EEprom)
   *
   * @param request Contains the different flags the user wants to set
   * @param response SUCCESS if success, MOTOR_ID_OUT_OF_RANGE if bad motor_id given
   *
   * @return false if motor_id is out of range
   */
  bool motor_system_controls_callback_(sr_robot_msgs::ChangeMotorSystemControls::Request &request,
                                       sr_robot_msgs::ChangeMotorSystemControls::Response &response);
};  // end class
}  // namespace shadow_robot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

#endif
