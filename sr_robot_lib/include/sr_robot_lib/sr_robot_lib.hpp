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

#include <ros_ethercat_model/robot_state.hpp>

#include <sr_robot_msgs/NullifyDemand.h>
#include <sr_robot_msgs/SetDebugData.h>

#include <sr_utilities/sr_math_utils.hpp>
#include <sr_utilities/calibration.hpp>
#include <sr_utilities/thread_safe_map.hpp>

#include <sr_self_test/sr_self_test.hpp>

#include "sr_robot_lib/sr_joint_motor.hpp"
#include "sr_robot_lib/generic_tactiles.hpp"

#include <sr_external_dependencies/types_for_external.h>
extern "C"
{
  #include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
  #include <sr_external_dependencies/external/0230_palm_edc_TS/0230_palm_edc_ethercat_protocol.h>
  #include <sr_external_dependencies/external/0320_palm_edc_muscle/0320_palm_edc_ethercat_protocol.h>
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
  template <class StatusType, class CommandType>
  class SrRobotLib
  {
  public:
    SrRobotLib(hardware_interface::HardwareInterface *hw);
    virtual ~SrRobotLib() {}

    /**
     * This function is called each time a new etherCAT message
     * is received in the sr06.cpp driver. It updates the joints_vector,
     * updating the different values, computing the calibrated joint
     * positions, etc... It also updates the tactile sensors values.
     *
     * @param status_data the received etherCAT message
     */
    virtual void update(StatusType* status_data) = 0;

    /**
     * Builds a command for the robot.
     *
     * @param command The command we're building.
     */
    virtual void build_command(CommandType* command) = 0;

    /**
     * Builds a command to demand information form the tactile sensors.
     *
     * @param command The command we're building.
     */
    void build_tactile_command(CommandType* command);

    /**
     * Reads the tactile information.
     *
     * @param status The status information that comes from the robot
     */
    void update_tactile_info(StatusType* status);

    /**
     * This function adds the diagnostics for the hand to the
     * multi diagnostic status published in sr06.cpp.
     */
    virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                         diagnostic_updater::DiagnosticStatusWrapper &d) = 0;

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
    boost::shared_ptr<tactiles::GenericTactiles<StatusType, CommandType> > tactiles;
    boost::shared_ptr<tactiles::GenericTactiles<StatusType, CommandType> > tactiles_init;

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

    ///Current update state of the sensors (initialization, operation..)
    operation_mode::device_update_state::DeviceUpdateState tactile_current_state;

    ros_ethercat_model::RobotState *hw_;

  protected:
    /// The vector containing all the robot joints.
    boost::ptr_vector<shadow_joints::Joint> joints_vector;

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
    void calibrate_joint(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data);


    /**
     * Calibrates and filters the position information (and computes velocity) for a give joint.
     * This method is called from the update method, each time a new message is received.
     *
     * @param joint_tmp The joint we process data from.
     * @param status_data The status information that comes from the robot
     * @param timestamp Timestamp of the data acquisition time
     */
    void process_position_sensor_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data, double timestamp);

    /**
     * Returns a pointer to the actuator state for a certain joint.
     * It checks the actuator type before accessing the state_ field, to avoid accessing the
     * base class state_ field which is not what we want
     *
     * @param joint_tmp The joint we want to get the actuator state from.
     *
     * @return a pointer to the actuator state
     */
    sr_actuator::SrActuatorState* get_joint_actuator_state(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp);

    /**
     * Reads the mapping between the sensors and the joints from the parameter server.
     *
     *
     * @return a vector (size of the number of joints) containing vectors (containing
     *         the sensors which are combined to form a given joint)
     */
    std::vector<shadow_joints::JointToSensor> read_joint_to_sensor_mapping();

    /**
     * Reads the calibration from the parameter server.
     *
     *
     * @return a calibration map
     */
    shadow_joints::CalibrationMap read_joint_calibration();

    /**
     * Simply reads the config from the parameter server.
     *
     * @param base_param string with the base name of the set of parameters to apply (found in the yaml file)
     * @param nb_data_defined number of data defined in the typedef
     * @param human_readable_data_types names of the types of messages (must match with those in the yaml file)
     * @param data_types the command values corresponding to every one of the names
     * @return A vector of UpdateConfig containing the type of data and the frequency
     *         at which we want to poll this data
     */
    std::vector<generic_updater::UpdateConfig> read_update_rate_configs(std::string base_param, int nb_data_defined, const char* human_readable_data_types[], const int32u data_types[]);


    /// A temporary calibration for a given joint.
    boost::shared_ptr<shadow_robot::JointCalibration> calibration_tmp;


    /// a ROS nodehandle (private naming, only inside the node namespace) to be able to advertise the Force PID service
    ros::NodeHandle nh_tilde;

    /// a ros nodehandle to be able to access resources out of the node namespace
    ros::NodeHandle nodehandle_;


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
    //static const int debug_mutex_lock_wait_time;
    //boost::shared_mutex debug_mutex;
    ros::NodeHandle node_handle;
    std_msgs::Int16 msg_debug;
#endif

    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> generic_sensor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> pst3_sensor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> biotac_sensor_update_rate_configs_vector;
    ///The update rate for each sensor information type
    std::vector<generic_updater::UpdateConfig> ubi0_sensor_update_rate_configs_vector;


    static const int nb_sensor_data;
    static const char* human_readable_sensor_data_types[];
    static const int32u sensor_data_types[];


    ///True if we want to set the demand to 0 (stop the controllers)
    bool nullify_demand_;
    ///The ROS service handler for nullifying the demand
    ros::ServiceServer nullify_demand_server_;

    /// It is run in a separate thread and calls the checkTests() method of the self_tests_. This avoids the tests blocking the main thread
    void checkSelfTests();

    boost::shared_ptr<SrSelfTest> self_tests_;

    ///Thread for running the tests in parallel when doing the tests on real hand
    boost::shared_ptr<boost::thread> self_test_thread_;
  };//end class
}//end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
