/**
 * @file   sr_muscle_robot_lib.hpp
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
 * @brief This is a generic robot library for Shadow Robot's muscle-actuated Hardware.
 *
 *
 */

#ifndef _SR_MUSCLE_ROBOT_LIB_HPP_
#define _SR_MUSCLE_ROBOT_LIB_HPP_

#include "sr_robot_lib/sr_robot_lib.hpp"

#define NUM_MUSCLE_DRIVERS      4

namespace shadow_robot
{
  template <class StatusType, class CommandType>
  class SrMuscleRobotLib : public SrRobotLib<StatusType, CommandType>
  {
  public:
    SrMuscleRobotLib(pr2_hardware_interface::HardwareInterface *hw);
    virtual ~SrMuscleRobotLib() {};

    /**
     * This function is called each time a new etherCAT message
     * is received in the sr06.cpp driver. It updates the joints_vector,
     * updating the different values, computing the calibrated joint
     * positions, etc... It also updates the tactile sensors values.
     *
     * @param status_data the received etherCAT message
     */
    void update(StatusType* status_data);

    /**
     * Builds a motor command: either send a torque demand or a configuration
     * demand if one is waiting.
     *
     * @param command The command we're building.
     */
    void build_command(ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND* command);

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


    ///Current update state of the motor (initialization, operation..)
    operation_mode::device_update_state::DeviceUpdateState muscle_current_state;


  protected:

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
                            std::vector<sr_actuator::SrGenericActuator*> actuators) = 0;

    /**
     * Read additional data from the latest message and stores it into the
     * joints_vector.
     *
     * @param joint_tmp The joint we want to read the data for.
     * @param status The status information that comes from the robot
     */
    void read_additional_muscle_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data);

    /**
     * Read additional data from the latest message and stores it into the
     * joints_vector.
     *
     * @param joint_tmp The joint we want to read the data for.
     * @param status The status information that comes from the robot
     */
    void read_muscle_driver_data(boost::ptr_vector<shadow_joints::MuscleDriver>::iterator muscle_driver_tmp, StatusType* status_data);

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
     * Decodes the pressure data for a certain muscle in a certain muscle driver from the status data structure
     */
    unsigned int get_muscle_pressure(int muscle_driver_id, int muscle_id, StatusType *status_data);

    inline void set_muscle_driver_data_received_flags(unsigned int msg_type, int muscle_driver_id);

    inline bool check_muscle_driver_data_received_flags();

    void set_valve_demand(uint8_t *muscle_data_byte_to_set, int8_t valve_value, uint8_t shift);


    boost::ptr_vector<shadow_joints::MuscleDriver> muscle_drivers_vector_;


    /**
     * The motor updater is used to create a correct command to send to the motor.
     * It's build_command() is called each time the SR06::packCommand()
     * is called.
     */
    boost::shared_ptr<generic_updater::MuscleUpdater<CommandType> > motor_updater_;


    ///contains a queue of muscle driver indexes to reset
    std::queue<short, std::list<short> > reset_muscle_driver_queue;



    ///The update rate for each muscle information
    std::vector<generic_updater::UpdateConfig> muscle_update_rate_configs_vector;

    /**
     *
     * A vector to store information about a certain message from the muscle driver. First in the pair is FROM_MUSCLE_DATA_TYPE
     * second in the pair is a bit mask where the bit in position id_muscle_driver tells if this data has been received from that muscle driver
     * It will only contain those FROM_MUSCLE_DATA_TYPE that are considered as initialization parameters
     */
    std::map<unsigned int, unsigned int> from_muscle_driver_data_received_flags_;



  };//end class
}//end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
