/**
 * @file   sr_robot_lib.hpp
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
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 *
 *
 */

#ifndef _SR_ROBOT_LIB_HPP_
#define _SR_ROBOT_LIB_HPP_

#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <deque>

//used to publish debug values
#include <std_msgs/Int16.h>

#include <sr_hardware_interface/sr_actuator.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <sr_robot_msgs/ForceController.h>
#include <sr_robot_msgs/NullifyDemand.h>
#include <sr_robot_msgs/SetDebugData.h>
#include <sr_robot_msgs/ControlType.h>
#include <sr_robot_msgs/ChangeControlType.h>
#include <sr_robot_msgs/MotorSystemControls.h>
#include <sr_robot_msgs/ChangeMotorSystemControls.h>

#include <sr_utilities/sr_math_utils.hpp>
#include <sr_utilities/calibration.hpp>
#include <sr_utilities/thread_safe_map.hpp>

#include <sr_self_test/sr_self_test.hpp>

#include "sr_robot_lib/sr_joint_motor.hpp"
#include "sr_robot_lib/motor_updater.hpp"
#include "sr_robot_lib/generic_tactiles.hpp"
#include "sr_robot_lib/motor_data_checker.hpp"

#include <sr_external_dependencies/types_for_external.h>
extern "C"
{
  #include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
  #include <sr_external_dependencies/external/simplemotor-bootloader/bootloader.h>
}

namespace operation_mode
{
  namespace robot_state
  {
    enum RobotState
    {
      INITIALIZATION,
      OPERATION,
      SHUTDOWN
    };
  }
}

namespace crc_unions
{
  typedef union
  {
    int16u word;
    int8u byte[2];
  } union16;
}

namespace shadow_robot
{
  //forward declaration of SrSelfTest
  class SrSelfTest;

  class SrRobotLib
  {
  public:
    SrRobotLib(pr2_hardware_interface::HardwareInterface *hw);
    virtual ~SrRobotLib() {};

    /**
     * This function is called each time a new etherCAT message
     * is received in the sr06.cpp driver. It updates the joints_vector,
     * updating the different values, computing the calibrated joint
     * positions, etc... It also updates the tactile sensors values.
     *
     * @param status_data the received etherCAT message
     */
    void update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data);

    /**
     * Builds a motor command: either send a torque demand or a configuration
     * demand if one is waiting.
     *
     * @param command The command we're building.
     */
    void build_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command);

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

    /**
     * Initiates the process to retrieve the initialization information from the sensors
     */
    void reinitialize_sensors();

    /**
     * This service is used to nullify the demand of the etherCAT
     *  hand. If the nullify_demand parameter is set to True,
     *  the demand sent to the robot will be 0, regardless of the
     *  computed effort demanded by the controller. If set to False,
     *  then the demand computed by the controllers will be sent to the motors.
     *
     * @param request contains the nullify_demand parameter
     * @param response empty
     *
     * @return always true as it can't fail
     */
    bool nullify_demand_callback( sr_robot_msgs::NullifyDemand::Request& request,
                                  sr_robot_msgs::NullifyDemand::Response& response );

    /// The map used to calibrate each joint.
    shadow_joints::CalibrationMap calibration_map;

    /**
     * This is a pointer to the tactile object. This pointer
     * will be instanciated during the initialization cycle,
     * depending on the type of sensors attached to the hand.
     */
    boost::shared_ptr<tactiles::GenericTactiles> tactiles;
    boost::shared_ptr<tactiles::GenericTactiles> tactiles_init;

    /**
     * Contains the idle time of the PIC communicating
     * via etherCAT with the host.
     */
    int main_pic_idle_time;

    /**
     * Contains the minimum idle time of the PIC communicating
     * via etherCAT with the host, this minimum is reset each
     * time a diagnostic is being published.
     */
    int main_pic_idle_time_min;

    ///Current update state of the motor (initialization, operation..)
    operation_mode::device_update_state::DeviceUpdateState motor_current_state;

    ///Current update state of the sensors (initialization, operation..)
    operation_mode::device_update_state::DeviceUpdateState tactile_current_state;

  protected:
    /// The vector containing all the robot joints.
    boost::ptr_vector<shadow_joints::Joint> joints_vector;

    /**
     * Initializes the joints_vector.
     *
     * @param joint_names A vector containing all the joints.
     * @param motor_ids A vector containing the corresponding motor indexes (-1 if no motor is associated).
     * @param joint_to_sensors The mapping between the joints and the sensors (e.g. FFJ0 = FFJ1+FFJ2)
     * @param actuators The actuators.
     */
    virtual void initialize(std::vector<std::string> joint_names,
                            std::vector<int> motor_ids,
                            std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                            std::vector<sr_actuator::SrActuator*> actuators) = 0;

    /**
     * Compute the calibrated position for the given joint. This method is called
     * from the update method, each time a new message is received.
     *
     * @param joint_tmp The joint we want to calibrate.
     */
    void calibrate_joint(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp);

    /**
     * Read additional data from the latest message and stores it into the
     * joints_vector.
     *
     * @param joint_tmp The joint we want to read teh data for.
     */
    void read_additional_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp);

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
    void generate_force_control_config(int motor_index, int max_pwm, int sg_left, int sg_right,
                                       int f, int p, int i, int d, int imax,
                                       int deadband, int sign);

    /**
     * The motor updater is used to create a correct command to send to the motor.
     * It's build_command() is called each time the SR06::packCommand()
     * is called.
     */
    boost::shared_ptr<generic_updater::MotorUpdater> motor_updater_;


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
    ///this index is used to iterate over the config we're sending.
    int config_index;

    ///contains a queue of motor indexes to reset
    std::queue<short, std::list<short> > reset_motors_queue;

    /// The current actuator.
    sr_actuator::SrActuator* actuator;
    /// The latest etherCAT message received.
    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data;
    /// A temporary calibration for a given joint.
    boost::shared_ptr<shadow_robot::JointCalibration> calibration_tmp;

    ///The index of the motor in all the 20 motors
    int motor_index_full;
    ///The index of the motor in the current message (from 0 to 9)
    int index_motor_in_msg;

    int8u crc_byte;
    int16u crc_result;
    int8u crc_i;

    /// a ROS nodehandle to be able to advertise the Force PID service
    ros::NodeHandle nh_tilde;

#ifdef DEBUG_PUBLISHER
    ///These publishers are useful for debugging
    static const int nb_debug_publishers_const;
    std::vector<ros::Publisher> debug_publishers;
    /**
     * A vector containing pairs:
     *  - associate a motor index
     *  - to a MOTOR_DATA
     *
     * This vector has the same size as the debug_publishers vector.
     */
    std::vector<boost::shared_ptr<std::pair<int, int> > > debug_motor_indexes_and_data;
    static const int debug_mutex_lock_wait_time;
    boost::shared_mutex debug_mutex;
    ros::NodeHandle node_handle;
    std_msgs::Int16 msg_debug;
#endif

    ///The update rate for each motor information
    std::vector<generic_updater::UpdateConfig> motor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> generic_sensor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> pst3_sensor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> biotac_sensor_update_rate_configs_vector;

    boost::shared_ptr<generic_updater::MotorDataChecker> motor_data_checker;

    ///True if we want to set the demand to 0 (stop the controllers)
    bool nullify_demand_;
    ///The ROS service handler for nullifying the demand
    ros::ServiceServer nullify_demand_server_;

    ///The current type of control (FORCE demand or PWM demand sent to the motors)
    sr_robot_msgs::ControlType control_type_;
    /**
     * Flag to signal that there has been a change in the value of control_type_ and certain actions are required.
     * The flag is set in the callback function of the change_control_type_ service.
     * The flag is checked in build_motor_command() and the necessary actions are taken there.
     * These actions involve calling services in the controller manager and all the active controllers. This is the
     * reason why we don't do it directly in the callback function. As we use a single thread to serve the callbacks,
     * doing so would cause a deadlock, thus we do it in the realtime loop thread instead.
     */
    bool control_type_changed_flag_;
    ///A service server used to change the control type on the fly.
    ros::ServiceServer change_control_type_;
    ///A mutual exclusion object to ensure that no command will be sent to the robot while a change in the control type (PWM or torque) is ongoing
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
    bool change_control_type_callback_( sr_robot_msgs::ChangeControlType::Request& request,
                                        sr_robot_msgs::ChangeControlType::Response& response );

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

    ///The Flag which will be sent to change the motor controls
    std::queue<std::vector<sr_robot_msgs::MotorSystemControls>, std::list<std::vector<sr_robot_msgs::MotorSystemControls> > > motor_system_control_flags_;
    ///A service server used to call the different motor system controls "buttons"
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
    bool motor_system_controls_callback_( sr_robot_msgs::ChangeMotorSystemControls::Request& request,
                                          sr_robot_msgs::ChangeMotorSystemControls::Response& response );

    boost::shared_ptr<SrSelfTest> self_tests_;
  };//end class
}//end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

