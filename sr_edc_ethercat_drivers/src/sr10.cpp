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
  * @file  sr10.cpp
  * @brief This is a ROS driver for Shadow Robot #10 EtherCAT product ID
  */


#include <sr_edc_ethercat_drivers/sr10.h>

#include <realtime_tools/realtime_publisher.h>

#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <std_msgs/Int16.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>
#include <memory>

#include <sr_utilities/sr_math_utils.hpp>

using std::string;
using std::stringstream;
using std::vector;

#include <sr_external_dependencies/types_for_external.h>


namespace is_edc_command_32_bits
{
// Assert if the EDC_COMMAND is 32bits (4 bytes) on the computer
  static_assert(sizeof(EDC_COMMAND) == 4);
}  // namespace is_edc_command_32_bits

/// The EtherCAT data structures for the SR10 driver
#define ETHERCAT_STATUS_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND)

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)

#define ETHERCAT_COMMAND_DATA_ADDRESS                   PALM_0250_ETHERCAT_COMMAND_DATA_ADDRESS
#define ETHERCAT_STATUS_DATA_ADDRESS                    PALM_0250_ETHERCAT_STATUS_DATA_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS        PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS         PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS


PLUGINLIB_EXPORT_CLASS(SR10, EthercatDevice);

/** 
  * SR10 ROS driver constructor.
  */
SR10::SR10()
        : zero_buffer_read(0),
          cycle_count(0),
          imu_scale_change_(false)
{
}

/** 
  *  Construct function, run at startup to set SyncManagers and FMMUs
  *
  *  The role of this function is to setup the SyncManagers and the FMMUs used by this EtherCAT slave.
  *  This slave is using two Mailboxes on two different memory areas.
  *
  *  Here we are setting up the way of communicating between ROS and the PIC32 using the EtherCAT protocol.
  *
  *  We communicate using Shared Memory areas.
  *
  *  The FMMUs are usefull to map the logical memory used by ROS to the Physical memory of the EtherCAT slave chip (ET1200 chip).
  *  So that the chip receiving the packet will know that the data at address 0x10000 is in reality to be written at physical address 0x1000 of the chip memory for example.
  *  It is the mapping between the EtherCAT bus address space and each slave's chip own memory address space.
  *
  *  The SyncManagers are usefull to give a safe way of accessing this Shared Memory, using a consumer / producer model. There are features like interrupts to tell the consumer
  *  that there is something to consume or to tell the producer that the Mailbox is empty and then ready to receive a new message.
  *
  *  - One Mailbox contains the commands, written by ROS, read by the PIC32
  *  - One Mailbox contains the status, written back by the PIC32, read by ROS
  *
  *  That's basically one Mailbox for upstream and one Mailbox for downstream.
  *
  * - The first Mailbox contains in fact two commands, one is the torque demand, the other is a CAN command used in CAN_DIRECT_MODE to communicate with the SimpleMotor for
  *   test purposes, or to reflash a new firmware in bootloading mode.
  *   This Mailbox is at logicial address 0x10000 and mapped via a FMMU to physical address 0x1000 (the first address of user memory)
  * - The second Mailbox contains in fact two status, they are the response of the two previously described commands. One is the status containing the joints data, sensor
  *   data, finger tips data and motor data. The other is the can command response in CAN_DIRECT_MODE. When doing a flashing in bootloading mode this is usually an acknowledgment
  *   from the bootloader. This Mailbox is at logical address 0x10038 and mapped via a FMMU to physical address 0x1038.
  *
  * This function sets the two private members command_size_ and status_size_ to be the size of each Mailbox.
  * It is important for these numbers to be accurate since they are used by the EthercatHardware class when manipulating the buffers.
  * If you need to have several commands like in this SR10 driver, put the sum of the size, same thing for the status.
  *
  * @param sh EtherCAT_SlaveHandler pointer
  * @param start_address the start address of the shared memory area  (this is the address of the first byte of the first Mailbox) 
  *
  */
void SR10::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  // Calls parent class construct function
  SrEdc::construct(sh, start_address, ETHERCAT_COMMAND_DATA_SIZE, ETHERCAT_STATUS_DATA_SIZE,
                   ETHERCAT_CAN_BRIDGE_DATA_SIZE,
                   ETHERCAT_COMMAND_DATA_ADDRESS, ETHERCAT_STATUS_DATA_ADDRESS,
                   ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS, ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS);

  ROS_INFO("Finished constructing the SR10 driver");
}

/**
  * Initialize the SR10 driver containg a the EtherCAT data and the SrMotorHandLib structures
  * necessary to build new commands and read current status from the Hand.
  *
  * @param hw the HardwareInterface pointer
  * @param allow_unprogrammed if true, the driver will not check if the Hand is programmed
  * @return 0 if successful, -1 otherwise
  */
int SR10::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int retval = SR0X::initialize(hw, allow_unprogrammed);

  if (retval != 0)
  {
    return retval;
  }

  // Cast the HardwareInterface pointer to a RobotState pointer
  hw_ = static_cast<ros_ethercat_model::RobotState *> (hw);

  // Create the SrMotorHandLib structure
  sr_hand_lib = std::shared_ptr<shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND> >(
          new shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS,
                  ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND>(hw, nodehandle_, nh_tilde_,
                                                                 device_id_, device_joint_prefix_));

  ROS_INFO("ETHERCAT_STATUS_DATA_SIZE      = %4d bytes", static_cast<int> (ETHERCAT_STATUS_DATA_SIZE));
  ROS_INFO("ETHERCAT_COMMAND_DATA_SIZE     = %4d bytes", static_cast<int> (ETHERCAT_COMMAND_DATA_SIZE));
  ROS_INFO("ETHERCAT_CAN_BRIDGE_DATA_SIZE  = %4d bytes", static_cast<int> (ETHERCAT_CAN_BRIDGE_DATA_SIZE));

  // Initialise the publisher for the extra analog inputs, gyroscope and accelerometer on the palm
  extra_analog_inputs_publisher.reset(
          new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nodehandle_, "palm_extras", 10));

  // Debug real time publisher: publishes the raw ethercat data
  debug_publisher = std::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug> >(
          new realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug>(nodehandle_, "debug_etherCAT_data", 4));

  debug_publisher->msg_.sensors.resize(SENSORS_NUM_0220 + 1);

  // Obtaining information from 10 motors (odd or even motors) for each Palm loop cycle
  debug_publisher->msg_.motor_data_packet_torque.resize(10);
  debug_publisher->msg_.motor_data_packet_misc.resize(10);

  std::string imu_name = device_joint_prefix_ + "imu";
  imu_state_ = hw_->getImuState(imu_name);

  // Imu initialization
  imu_state_->data_.orientation[0] = 0.0; imu_state_->data_.orientation[1] = 0.0;
  imu_state_->data_.orientation[2] = 0.0; imu_state_->data_.orientation[3] = 1.0;

  std::fill(imu_state_->data_.linear_acceleration_covariance,
            imu_state_->data_.linear_acceleration_covariance + 9, 0.0);
  std::fill(imu_state_->data_.angular_velocity_covariance,
            imu_state_->data_.angular_velocity_covariance + 9, 0.0);
  std::fill(imu_state_->data_.orientation_covariance,
            imu_state_->data_.orientation_covariance + 9, 0.0);

  // Broadcast /set_gyr_scale and /set_acc_scale services
  imu_gyr_scale_server_ =  nodehandle_.advertiseService
    <sr_robot_msgs::SetImuScale::Request, sr_robot_msgs::SetImuScale::Response>
    ("/" + imu_name + "/set_gyr_scale", boost::bind(&SR10::imu_scale_callback_, this, _1, _2, "gyr"));
  imu_acc_scale_server_ =  nodehandle_.advertiseService
    <sr_robot_msgs::SetImuScale::Request, sr_robot_msgs::SetImuScale::Response>
    ("/" + imu_name + "/set_acc_scale", boost::bind(&SR10::imu_scale_callback_, this, _1, _2, "acc"));

  // Initialize default IMU scaling values
  ros::param::param<int>("/" + imu_name + "/acc_scale", imu_scale_acc_, 0);
  ros::param::param<int>("/" + imu_name + "/gyr_scale", imu_scale_gyr_, 0);

  // and set the flag to true so that the IMU data is scaled
  imu_scale_change_ = true;

  // Adjust extra_analog_msg_ data labels and sizes
  extra_analog_msg_.layout.dim.resize(3);
  extra_analog_msg_.data.resize(3 + 3 + 4);
  extra_analog_msg_.layout.dim[0].label = "accelerometer";
  extra_analog_msg_.layout.dim[0].size = 3;
  extra_analog_msg_.layout.dim[1].label = "gyrometer";
  extra_analog_msg_.layout.dim[1].size = 3;
  extra_analog_msg_.layout.dim[2].label = "analog_inputs";
  extra_analog_msg_.layout.dim[2].size = 4;

  /// A counter for the number of frames received with status EDC_COMMAND_INVALID
  invalid_frame_counter_ = 0;

  return retval;
}

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
void SR10::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &diagnostic_vector, unsigned char *buffer)
{
  diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status(diagnostic_status_);

  stringstream name;
  string prefix = device_id_.empty() ? device_id_ : (device_id_ + " ");
  diagnostic_status.name = prefix + "EtherCAT Dual CAN Palm";
  diagnostic_status.summary(diagnostic_status.OK, "OK");
  stringstream hardware_id;
  hardware_id << sh_->get_product_code() << "-" << sh_->get_serial();
  diagnostic_status.hardware_id = hardware_id.str();

  diagnostic_status.clear();
  diagnostic_status.addf("Position", "%02d", sh_->get_ring_position());
  diagnostic_status.addf("Product Code", "%d", sh_->get_product_code());
  diagnostic_status.addf("Serial Number", "%d", sh_->get_serial());
  diagnostic_status.addf("Revision", "%d", sh_->get_revision());
  diagnostic_status.addf("Counter", "%d", ++counter_);
  diagnostic_status.addf("Invalid Frames Received Counter", "%d", invalid_frame_counter_);

  diagnostic_status.addf("PIC idle time (in microsecs)", "%d", sr_hand_lib->main_pic_idle_time);
  diagnostic_status.addf("Min PIC idle time (since last diagnostics)", "%d", sr_hand_lib->main_pic_idle_time_min);
  // reset the idle time min to a big number, to get a fresh number on next diagnostic
  sr_hand_lib->main_pic_idle_time_min = 1000;

  this->ethercatDiagnostics(diagnostic_status, 2);
  diagnostic_vector.push_back(diagnostic_status);

  // Add the diagnostics from the hand
  sr_hand_lib->add_diagnostics(diagnostic_vector, diagnostic_status);

  // Add the diagnostics from the tactiles
  if (sr_hand_lib->tactiles != NULL)
  {
    sr_hand_lib->tactiles->add_diagnostics(diagnostic_vector, diagnostic_status);
  }
}

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
void SR10::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  SrEdc::packCommand(buffer, halt, reset);

  ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND *command =
                                    reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND *>(buffer);
  ETHERCAT_CAN_BRIDGE_DATA *message =
                                    reinterpret_cast<ETHERCAT_CAN_BRIDGE_DATA *>(buffer + ETHERCAT_COMMAND_DATA_SIZE);

  if (!flashing)
  {
    command->EDC_command = EDC_COMMAND_SENSOR_DATA;
  }
  else
  {
    command->EDC_command = EDC_COMMAND_CAN_DIRECT_MODE;
  }

  if (imu_scale_change_)
  {
    command->imu_command.command = IMU_COMMAND_SET_SCALE;
    command->imu_command.argument[0] = imu_scale_acc_;
    command->imu_command.argument[1] = imu_scale_gyr_;
    imu_scale_change_ = false;
  }
  else
  {
    command->imu_command.command = IMU_COMMAND_NONE;
  }

  // Request for the different sensors' (including tactile) and motors' status information.
  // It alternates between even and uneven motors everytime.
  // It also builds the next control command to send to the motors (e.g. torque control)
  sr_hand_lib->build_command(command);

  ROS_DEBUG("Sending command : Type : 0x%02X ; data : 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X "
                    "0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X",
            command->to_motor_data_type,
            command->motor_data[0],
            command->motor_data[1],
            command->motor_data[2],
            command->motor_data[3],
            command->motor_data[4],
            command->motor_data[5],
            command->motor_data[6],
            command->motor_data[7],
            command->motor_data[8],
            command->motor_data[9],
            command->motor_data[10],
            command->motor_data[11],
            command->motor_data[12],
            command->motor_data[13],
            command->motor_data[14],
            command->motor_data[15],
            command->motor_data[16],
            command->motor_data[17],
            command->motor_data[18],
            command->motor_data[19]);

  build_CAN_message(message);
}

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
bool SR10::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS *status_data =
          reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS *>(this_buffer + command_size_);
  ETHERCAT_CAN_BRIDGE_DATA *can_data = reinterpret_cast<ETHERCAT_CAN_BRIDGE_DATA *>(this_buffer + command_size_ +
                                                                                    ETHERCAT_STATUS_DATA_SIZE);
  // number of packets received since the start of the driver
  static unsigned int num_rxed_packets = 0;

  ++num_rxed_packets;

  // publishes the debug information (a slightly formatted version of the incoming ethercat packet):
  if (debug_publisher->trylock())
  {
    debug_publisher->msg_.header.stamp = ros::Time::now();

    for (unsigned int index = 0; index < SENSORS_NUM_0220 + 1; ++index)
    {
      debug_publisher->msg_.sensors[index] = (status_data->sensors[index]);
    }

    debug_publisher->msg_.motor_data_type.data = static_cast<int> (status_data->motor_data_type);
    debug_publisher->msg_.which_motors = status_data->which_motors;
    debug_publisher->msg_.which_motor_data_arrived = status_data->which_motor_data_arrived;
    debug_publisher->msg_.which_motor_data_had_errors = status_data->which_motor_data_had_errors;

    // Getting information from 10 motors at a time
    for (unsigned int index = 0; index < 10; ++index)
    {
      debug_publisher->msg_.motor_data_packet_torque[index] = (status_data->motor_data_packet[index].torque);
      debug_publisher->msg_.motor_data_packet_misc[index] = (status_data->motor_data_packet[index].misc);
    }

    debug_publisher->msg_.tactile_data_type = static_cast<int32u>(status_data->tactile_data_type);
    debug_publisher->msg_.tactile_data_valid = static_cast<int16u> (status_data->tactile_data_valid);

    debug_publisher->msg_.idle_time_us = status_data->idle_time_us;

    debug_publisher->unlockAndPublish();
  }

  if (status_data->EDC_command == EDC_COMMAND_INVALID)
  {
    // received empty message: the pic is not writing to its mailbox.
    ++zero_buffer_read;
    float percentage_packet_loss = 100.f * (static_cast<float>(zero_buffer_read) /
                                            static_cast<float>(num_rxed_packets));

    invalid_frame_counter_++;
    ROS_DEBUG("Reception error detected : %d errors out of %d rxed packets (%2.3f%%) ; idle time %dus",
              zero_buffer_read, num_rxed_packets, percentage_packet_loss, status_data->idle_time_us);
    return true;
  }

  // We've received a coherent/valid message.
  // Read IMU status and update internal structures
  readImu(status_data);

  // Update the library (positions, diagnostics values, actuators, etc...)
  // with the received information received from the master
  sr_hand_lib->update(status_data);

  // Now publish the data at 100Hz (every 10 cycles)
  if (cycle_count >= 10)
  {
    // Publish additional data (accelerometer / gyroscope / analog inputs)
    extra_analog_msg_.data[0] = (int16_t) status_data->sensors[ACCX];
    extra_analog_msg_.data[1] = (int16_t) status_data->sensors[ACCY];
    extra_analog_msg_.data[2] = (int16_t) status_data->sensors[ACCZ];

    extra_analog_msg_.data[3] = (int16_t) status_data->sensors[GYRX];
    extra_analog_msg_.data[4] = (int16_t) status_data->sensors[GYRY];
    extra_analog_msg_.data[5] = (int16_t) status_data->sensors[GYRZ];

    extra_analog_msg_.data[6] = status_data->sensors[ANA0];
    extra_analog_msg_.data[7] = status_data->sensors[ANA1];
    extra_analog_msg_.data[8] = status_data->sensors[ANA2];
    extra_analog_msg_.data[9] = status_data->sensors[ANA3];

    if (extra_analog_inputs_publisher->trylock())
    {
      extra_analog_inputs_publisher->msg_ = extra_analog_msg_;
      extra_analog_inputs_publisher->unlockAndPublish();
    }

    cycle_count = 0;
  }
  ++cycle_count;


  // If we're flashing, check is the packet has been acked
  if (flashing & !can_packet_acked)
  {
    if (can_data_is_ack(can_data))
    {
      can_packet_acked = true;
    }
  }

  return true;
}

/**
  * This function will call the reinitialization function for the boards attached to the CAN bus
  */
void SR10::reinitialize_boards()
{
  // Reinitialize motors information
  sr_hand_lib->reinitialize_motors();
}

/**
  * Given the identifier for a certain board (motor board/ muscle driver) determines the right value
  * for the CAN bus and the ID of the board in that CAN bus.
  *
  * @param board_id the unique identifier for the board
  * @param can_bus pointer to the can bus number we want to determine
  * @param board_can_id pointer to the board id we want to determine
*/
void SR10::get_board_id_and_can_bus(int board_id, int *can_bus, unsigned int *board_can_id)
{
  // We're using 2 can busses,
  // if motor id is between 0 and 9, then we're using the can_bus 1
  // else, we're using the can bus 2.
  int motor_id_tmp = board_id;
  if (motor_id_tmp > 9)
  {
    motor_id_tmp -= 10;
    *can_bus = 2;
  }
  else
  {
    *can_bus = 1;
  }

  *board_can_id = motor_id_tmp;
}

/**
  * New scales to be set on IMU's gyroscope or accelerometer.
  * Sets flag (imu_scale_gyr_) indicating that a new "scalling" command should be sent to the IMU.
  *
  * @param request service request containing the new scaling factor to be set for the IMU's gyroscope or accelerometer
  * @param response service response (empty in this case)
  * @param which pointer to a string that selects IMU's gyroscope or accelerometer
  * @return true if the new scaling factor was set successfully, false otherwise  
  */
bool SR10::imu_scale_callback_(sr_robot_msgs::SetImuScale::Request &request,
                               sr_robot_msgs::SetImuScale::Response &response,
                               const char *which)
{
  if (request.scale == 0 || request.scale == 1 || request.scale == 2)
  {
    if (which == "acc")
    {
      imu_scale_acc_ = request.scale;
    }
    else if (which == "gyr")
    {
      imu_scale_gyr_ = request.scale;
    }

    imu_scale_change_ = true;
    return true;
  }
  else
  {
    ROS_WARN_STREAM("Tried to set illegal value: " << (int) request.scale);
    return false;
  }
}

/** 
  * This funcion reads the ethercat status and fills the imu_state with the relevant values.
  *
  * @param status_data pointer to the received EtherCAT Status data
  */
void SR10::readImu(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS * status_data)
{
  double acc_multiplier = 1 << imu_scale_acc_;
  double gyr_multiplier = 1 << imu_scale_gyr_;

  int16_t zero_catch = 0;

  for (size_t index = 0; index < 2; ++index)
  {
    if (status_data->sensors[ACCX + index] == 0)
    {
      ++zero_catch;
    }
    if (status_data->sensors[GYRX + index] == 0)
    {
      ++zero_catch;
    }
  }
  // It seems that this was an attempt to identify an invalid status sent by the IMU
  // However, it is not clear that "all zeros" would not be a valid IMU state
  if (zero_catch < 6)
  {
    imu_state_->data_.linear_acceleration[0] = acc_multiplier * static_cast<int16s>(status_data->sensors[ACCX]);
    imu_state_->data_.linear_acceleration[1] = acc_multiplier * static_cast<int16s>(status_data->sensors[ACCY]);
    imu_state_->data_.linear_acceleration[2] = acc_multiplier * static_cast<int16s>(status_data->sensors[ACCZ]);

    imu_state_->data_.angular_velocity[0] = gyr_multiplier * static_cast<int16s>(status_data->sensors[GYRX]);
    imu_state_->data_.angular_velocity[1] = gyr_multiplier * static_cast<int16s>(status_data->sensors[GYRY]);
    imu_state_->data_.angular_velocity[2] = gyr_multiplier * static_cast<int16s>(status_data->sensors[GYRZ]);
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
