/* 
* Copyright 2023, 2024 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/
/**
  * @file  sr10.h
  * @brief This is a ROS driver for Shadow Robot #10 EtherCAT product ID
  */

#ifndef SR_EDC_ETHERCAT_DRIVERS_SR10_H
#define SR_EDC_ETHERCAT_DRIVERS_SR10_H

#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <sr_edc_ethercat_drivers/sr_edc.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <sr_robot_msgs/SetImuScale.h>
#include <sr_robot_msgs/SimpleMotorFlasher.h>
#include <pthread.h>
#include <bfd.h>
#include <map>
#include <vector>
#include <memory>

#include <sr_robot_lib/sr_motor_hand_lib.hpp>

#include <sr_robot_msgs/EthercatDebug.h>

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/0250_palm_edc_IMU_MST/0250_palm_edc_IMU_ethercat_protocol.h>
}

/** 
  * Class that contains all functions and member variables relevant to the SR10 ROS drivers.
  * It inherits from the SrEdc class.
  */  
class SR10 :
        public SrEdc
{
public:
  /**
    * SR10 ROS driver constructor.
    */
  SR10();

  /** 
    * Construct function, run at startup to set SyncManagers and FMMUs
    * It sets up the SyncManagers and the FMMUs used by this EtherCAT client.
    * It sets up two Mailboxes on two different memory areas (for the command requests and state responses)
    *
    * @param sh EtherCAT_SlaveHandler pointer
    * @param start_address the start address of the shared memory area  (this is the address of the first byte of the first Mailbox) 
    *
    */
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);

  /**
    * Initialize the SR10 driver containg a the necessary EtherCAT data and the SrMotorHandLib structures
    * necessary to build new commands and read current status from the Hand.
    *
    * @param hw the HardwareInterface pointer
    * @param allow_unprogrammed if true, the driver will not check if the Hand is programmed
    * @return 0 if successful, -1 otherwise
    */
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed = true);

  /**
    *  This function provides diagnostics data that can be displayed by
    *  the runtime_monitor node. We use the mutliDiagnostics as it publishes
    *  the diagnostics for each motors.
    *
    * @param diagnostic_vector the vector of diagnostic messages
    * @param buffer is a pointer to some string. Not used in SR10.
    *        Can we remove this? It would involve also removing
    *        it from ethercat_hardware (EthercatHardwareDiagnosticsPublisher::publishDiagnostics)
    */
  virtual void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &diagnostic_vector, unsigned char *buffer);

  /** 
    *  Packs the commands before sending them to the EtherCAT bus
    *
    *  This is one of the most important functions of this driver.
    *  This function is called each millisecond (1 kHz freq) by the EthercatHardware::update() function
    *  in the controlLoop() of the ros_etherCAT node. buffer is cast into the Ethercat structures related
    *  to SR10 drivers. The command to be sent to the hand is populated with relevant data.
    *  There is a second command which is meant to bo sent to the CAN busses. The memory address corresponding 
    *  to the beggining of the second command can be found by shifting the pointer
    *  by the size of the first command.
    * 
    * @param buffer is a pointer to an array containing 2 commands to be sent to the hand (and CAN busses).
    *        The buffer has been allocated with command_size_ bytes, which is the sum of the two command size,
    *        so we have to put the two commands one next to the other.
    *        These are then sent via EtherCAT.
    * @param halt   if true, it will disable actuator, usually by disabling H-bridge
    * @param reset  if true, it will clear diagnostic error conditions device safety disable
    */
  virtual void packCommand(unsigned char *buffer, bool halt, bool reset);

  /** 
    *  This function handles receiving data from the EtherCAT bus
    *
    *  It allows the driver to get the data present on the EtherCAT bus and update its internal structures.
    *
    *  It gives us access to the logical memory instantiated during construct().
    *  We access the data sent by PIC32 in a fashon similar to packCommand() (through direct memeory address).
    *  
    *  @param this_buffer The data just being received by EtherCAT
    *  @param prev_buffer The previous data received by EtherCAT
    */
  virtual bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

protected:
  /// Extra analog inputs real time publisher (+ accelerometer and gyroscope)
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > extra_analog_inputs_publisher;

  /**
    * This function will call the reinitialization function for the boards attached to the CAN bus
    */
  virtual void reinitialize_boards();

  /**
    * Given the identifier for a certain board (motor board/ muscle driver) determines the right value
    * for the CAN bus and the ID of the board in that CAN bus.
    *
    * @param board_id the unique identifier for the board
    * @param can_bus pointer to the can bus number we want to determine
    * @param board_can_id pointer to the board id we want to determine
    */
  virtual void get_board_id_and_can_bus(int board_id, int *can_bus, unsigned int *board_can_id);

/**
  * New scales to be set on IMU's gyroscope or accelerometer.
  * Sets flag (imu_scale_gyr_) indicating that a new "scalling" command should be sent to the IMU.
  *
  * @param request service request containing the new scaling factor to be set for the IMU's gyroscope or accelerometer
  * @param response service response (empty in this case)
  * @param which pointer to a string that selects IMU's gyroscope or accelerometer
  * @return true if the new scaling factor was set successfully, false otherwise  
  */
  bool imu_scale_callback_(sr_robot_msgs::SetImuScale::Request &request,
                           sr_robot_msgs::SetImuScale::Response &response,
                           const char *which);


private:
  /// Services to set the IMU's gyroscope and accelerometer scales
  ros::ServiceServer imu_gyr_scale_server_;
  ros::ServiceServer imu_acc_scale_server_;

  /// Counter for the number of invalid Status replies we recieve.
  unsigned int zero_buffer_read;

  /// Robot state interface
  ros_ethercat_model::RobotState * hw_;

  /// IMU state interface
  ros_ethercat_model::ImuState * imu_state_;
  /// Pointer to etherCAT hand. Contains the necessary structures to build the (etherCAT) commands and read the status
  std::shared_ptr<shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS,
                                               ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND>> sr_hand_lib;

  // IMU scaling variables
  int imu_scale_gyr_;
  int imu_scale_acc_;
  bool imu_scale_change_;
  std_msgs::Float64MultiArray extra_analog_msg_;

  /// A counter used to publish the tactiles at 100Hz. Counts 10 cycles, then reset the cycle_count to 0.
  int16_t cycle_count;

  /// A counter for the number of frames received with status EDC_COMMAND_INVALID
  int invalid_frame_counter_;

  /** 
    * This funcion reads the ethercat status and fills the imu_state with the relevant values.
    *
    * @param status_data pointer to the received EtherCAT Status data
    */
  void readImu(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS * status_data);

  /// Debug real time publisher: publishes the raw ethercat data
  std::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug> > debug_publisher;
};


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif  // SR_EDC_ETHERCAT_DRIVERS_SR10_H
