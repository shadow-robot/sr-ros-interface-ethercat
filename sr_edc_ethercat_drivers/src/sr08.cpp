/**
 * @file   sr08.cpp
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
 *         Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
 * @date   Tue May 07 13:33:30 2013
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
 * @brief This is a ROS driver for Shadow Robot #8 EtherCAT product ID
 *
 *
 */


#include <sr_edc_ethercat_drivers/sr08.h>

#include <realtime_tools/realtime_publisher.h>

#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <boost/foreach.hpp>
#include <std_msgs/Int16.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>

#include <sr_utilities/sr_math_utils.hpp>

using std::string;
using std::stringstream;
using std::vector;

#include <sr_external_dependencies/types_for_external.h>

#include <boost/static_assert.hpp>

namespace is_edc_command_32_bits
{
// check is the EDC_COMMAND is 32bits on the computer
// if not, fails
  BOOST_STATIC_ASSERT(sizeof(EDC_COMMAND) == 4);
}  // namespace is_edc_command_32_bits

#define ETHERCAT_STATUS_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND)

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)

#define ETHERCAT_COMMAND_DATA_ADDRESS                   PALM_0230_ETHERCAT_COMMAND_DATA_ADDRESS
#define ETHERCAT_STATUS_DATA_ADDRESS                    PALM_0230_ETHERCAT_STATUS_DATA_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS        PALM_0230_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS         PALM_0230_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS


PLUGINLIB_EXPORT_CLASS(SR08, EthercatDevice);

/** \brief Constructor of the SR08 driver
 *
 *  This is the Constructor of the driver. We
 *  initialize a few boolean values, a mutex
 *  and create the Bootloading service.
 */
SR08::SR08()
        : zero_buffer_read(0),
          cycle_count(0)
{
  /*
    ROS_INFO("There are %d sensors", nb_sensors_const);
    ROS_INFO(     "device_pub_freq_const = %d", device_pub_freq_const      );
    ROS_INFO(        "ros_pub_freq_const = %d", ros_pub_freq_const         );
    ROS_INFO(            "max_iter_const = %d", max_iter_const             );
    ROS_INFO(          "nb_sensors_const = %d", nb_sensors_const           );
    ROS_INFO("nb_publish_by_unpack_const = %d", nb_publish_by_unpack_const );
   */
}

/** \brief Construct function, run at startup to set SyncManagers and FMMUs
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
 * If you need to have several commands like in this SR08 driver, put the sum of the size, same thing for the status.
 *
 */
void SR08::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  ROS_ASSERT(ETHERCAT_STATUS_0230_AGREED_SIZE == ETHERCAT_STATUS_DATA_SIZE);
  ROS_ASSERT(ETHERCAT_COMMAND_0230_AGREED_SIZE == ETHERCAT_COMMAND_DATA_SIZE);

  SrEdc::construct(sh, start_address, ETHERCAT_COMMAND_DATA_SIZE, ETHERCAT_STATUS_DATA_SIZE,
                   ETHERCAT_CAN_BRIDGE_DATA_SIZE,
                   ETHERCAT_COMMAND_DATA_ADDRESS, ETHERCAT_STATUS_DATA_ADDRESS,
                   ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS, ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS);

  ROS_INFO("Finished constructing the SR08 driver");
}

/**
 *
 */
int SR08::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int retval = SR0X::initialize(hw, allow_unprogrammed);

  if (retval != 0)
  {
    return retval;
  }

  sr_hand_lib = boost::shared_ptr<shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND> >(
          new shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS,
                  ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>(hw, nodehandle_, nh_tilde_,
                                                                 device_id_, device_joint_prefix_));

  ROS_INFO("ETHERCAT_STATUS_DATA_SIZE      = %4d bytes", static_cast<int> (ETHERCAT_STATUS_DATA_SIZE));
  ROS_INFO("ETHERCAT_COMMAND_DATA_SIZE     = %4d bytes", static_cast<int> (ETHERCAT_COMMAND_DATA_SIZE));
  ROS_INFO("ETHERCAT_CAN_BRIDGE_DATA_SIZE  = %4d bytes", static_cast<int> (ETHERCAT_CAN_BRIDGE_DATA_SIZE));

  // initialise the publisher for the extra analog inputs, gyroscope and accelerometer on the palm
  extra_analog_inputs_publisher.reset(
          new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nodehandle_, "palm_extras", 10));


  // Debug real time publisher: publishes the raw ethercat data
  debug_publisher = boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug> >(
          new realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug>(nodehandle_, "debug_etherCAT_data", 4));
  return retval;
}

/** \brief This function gives some diagnostics data
 *
 *  This function provides diagnostics data that can be displayed by
 *  the runtime_monitor node. We use the mutliDiagnostics as it publishes
 *  the diagnostics for each motors.
 */
void SR08::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer)
{
  diagnostic_updater::DiagnosticStatusWrapper &d(diagnostic_status_);

  stringstream name;
  string prefix = device_id_.empty() ? device_id_ : (device_id_ + " ");
  d.name = prefix + "EtherCAT Dual CAN Palm";
  d.summary(d.OK, "OK");
  stringstream hwid;
  hwid << sh_->get_product_code() << "-" << sh_->get_serial();
  d.hardware_id = hwid.str();

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product Code", "%d", sh_->get_product_code());
  d.addf("Serial Number", "%d", sh_->get_serial());
  d.addf("Revision", "%d", sh_->get_revision());
  d.addf("Counter", "%d", ++counter_);

  d.addf("PIC idle time (in microsecs)", "%d", sr_hand_lib->main_pic_idle_time);
  d.addf("Min PIC idle time (since last diagnostics)", "%d", sr_hand_lib->main_pic_idle_time_min);
  // reset the idle time min to a big number, to get a fresh number on next diagnostic
  sr_hand_lib->main_pic_idle_time_min = 1000;

  this->ethercatDiagnostics(d, 2);
  vec.push_back(d);

  // Add the diagnostics from the hand
  sr_hand_lib->add_diagnostics(vec, d);

  // Add the diagnostics from the tactiles
  if (sr_hand_lib->tactiles != NULL)
  {
    sr_hand_lib->tactiles->add_diagnostics(vec, d);
  }
}

/** \brief packs the commands before sending them to the EtherCAT bus
 *
 *  This is one of the most important functions of this driver.
 *  This function is called each millisecond (1 kHz freq) by the EthercatHardware::update() function
 *  in the controlLoop() of the ros_etherCAT node.
 *
 *  This function is called with a buffer as a parameter, the buffer provided is where we write the commands to send via EtherCAT.
 *
 *  We just cast the buffer to our structure type, fill the structure with our data, then add the structure size to the buffer address to shift into memory and access the second command.
 *  The buffer has been allocated with command_size_ bytes, which is the sum of the two command size, so we have to put the two commands one next to the other.
 *  In fact we access the buffer using this kind of code : \code
 *  ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND  *command = (ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND *)buffer;
 *  ETHERCAT_CAN_BRIDGE_DATA                       *message = (ETHERCAT_CAN_BRIDGE_DATA *)(buffer + ETHERCAT_COMMAND_DATA_SIZE);
 *  \endcode
 */
void SR08::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  SrEdc::packCommand(buffer, halt, reset);

  ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND *command =
          reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND *>(buffer);
  ETHERCAT_CAN_BRIDGE_DATA *message = reinterpret_cast<ETHERCAT_CAN_BRIDGE_DATA *>(buffer + ETHERCAT_COMMAND_DATA_SIZE);

  if (!flashing)
  {
    command->EDC_command = EDC_COMMAND_SENSOR_DATA;
  }
  else
  {
    command->EDC_command = EDC_COMMAND_CAN_DIRECT_MODE;
  }

  // alternate between even and uneven motors
  // and ask for the different informations.
  sr_hand_lib->build_command(command);

  // @todo For the moment the aux_data_type in the commend will be fixed here. This is for convenience,
  // before we separate the aux data (and the prox_mid data) from the UBIO sensor type in the driver.
  // After that, this should be done in the aux_data_updater (or something similar ) in the driver
  command->aux_data_type = TACTILE_SENSOR_TYPE_MCP320x_TACTILE;

  ROS_DEBUG(
          "Sending command : Type : 0x%02X ; data : 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X"
                  " 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X",
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

/** \brief This functions receives data from the EtherCAT bus
 *
 *  This function allows the driver to get the data present on the EtherCAT bus and intended for us.
 *
 *  It gives us access to the logical memory registered during the construct().
 *
 *  In order to be able to do differentials two buffers are kept, this_buffer is the actual data that has just been received
 *  and prev_buffer is the previous buffer received from the EtherCAT bus.
 *
 *  We access the data sent by PIC32 here using the same tricks we used in packCommand().
 *  \code
 *  ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS *tbuffer = (ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS *)(this_buffer + command_size_);
 *  ETHERCAT_CAN_BRIDGE_DATA *can_data = (ETHERCAT_CAN_BRIDGE_DATA *)(this_buffer + command_size_ + ETHERCAT_STATUS_DATA_SIZE);
 *  \endcode
 *
 * @param this_buffer The data just being received by EtherCAT
 * @param prev_buffer The previous data received by EtherCAT
 */
bool SR08::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS *status_data =
          reinterpret_cast<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS *>(this_buffer + command_size_);
  ETHERCAT_CAN_BRIDGE_DATA *can_data =
          reinterpret_cast<ETHERCAT_CAN_BRIDGE_DATA *>(this_buffer + command_size_ + ETHERCAT_STATUS_DATA_SIZE);
  //  int16u                                        *status_buffer = (int16u*)status_data;
  static unsigned int num_rxed_packets = 0;

  ++num_rxed_packets;


  // publishes the debug information (a slightly formatted version of the incoming ethercat packet):
  if (debug_publisher->trylock())
  {
    debug_publisher->msg_.header.stamp = ros::Time::now();

    debug_publisher->msg_.sensors.clear();
    for (unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    {
      debug_publisher->msg_.sensors.push_back(status_data->sensors[i]);
    }

    debug_publisher->msg_.motor_data_type.data = static_cast<int> (status_data->motor_data_type);
    debug_publisher->msg_.which_motors = status_data->which_motors;
    debug_publisher->msg_.which_motor_data_arrived = status_data->which_motor_data_arrived;
    debug_publisher->msg_.which_motor_data_had_errors = status_data->which_motor_data_had_errors;

    debug_publisher->msg_.motor_data_packet_torque.clear();
    debug_publisher->msg_.motor_data_packet_misc.clear();
    for (unsigned int i = 0; i < 10; ++i)
    {
      debug_publisher->msg_.motor_data_packet_torque.push_back(status_data->motor_data_packet[i].torque);
      debug_publisher->msg_.motor_data_packet_misc.push_back(status_data->motor_data_packet[i].misc);
    }

    debug_publisher->msg_.tactile_data_type = static_cast<unsigned int> (
            static_cast<int32u>(status_data->tactile_data_type));
    debug_publisher->msg_.tactile_data_valid = static_cast<unsigned int> (
            static_cast<int16u> (status_data->tactile_data_valid));
    debug_publisher->msg_.tactile.clear();
    for (unsigned int i = 0; i < 5; ++i)
    {
      debug_publisher->msg_.tactile.push_back(
              static_cast<unsigned int> (static_cast<int16u> (status_data->tactile[i].word[0])));
    }

    debug_publisher->msg_.idle_time_us = status_data->idle_time_us;

    debug_publisher->unlockAndPublish();
  }

  if (status_data->EDC_command == EDC_COMMAND_INVALID)
  {
    // received empty message: the pic is not writing to its mailbox.
    ++zero_buffer_read;
    float percentage_packet_loss = 100.f * (static_cast<float>(zero_buffer_read) /
            static_cast<float>(num_rxed_packets));

    ROS_DEBUG("Reception error detected : %d errors out of %d rxed packets (%2.3f%%) ; idle time %dus",
              zero_buffer_read, num_rxed_packets, percentage_packet_loss, status_data->idle_time_us);
    return true;
  }

  // We received a coherent message.
  // Update the library (positions, diagnostics values, actuators, etc...)
  // with the received information
  sr_hand_lib->update(status_data);

  // Now publish the additional data at 100Hz (every 10 cycles)
  if (cycle_count >= 10)
  {
    // publish tactiles if we have them
    if (sr_hand_lib->tactiles != NULL)
    {
      sr_hand_lib->tactiles->publish();
    }

    // And we also publish the additional data (accelerometer / gyroscope / analog inputs)
    std_msgs::Float64MultiArray extra_analog_msg;
    extra_analog_msg.layout.dim.resize(3);
    extra_analog_msg.data.resize(3 + 3 + 4);
    std::vector<double> data;

    extra_analog_msg.layout.dim[0].label = "accelerometer";
    extra_analog_msg.layout.dim[0].size = 3;
    extra_analog_msg.data[0] = status_data->sensors[ACCX];
    extra_analog_msg.data[1] = status_data->sensors[ACCY];
    extra_analog_msg.data[2] = status_data->sensors[ACCZ];

    extra_analog_msg.layout.dim[1].label = "gyrometer";
    extra_analog_msg.layout.dim[1].size = 3;
    extra_analog_msg.data[3] = status_data->sensors[GYRX];
    extra_analog_msg.data[4] = status_data->sensors[GYRY];
    extra_analog_msg.data[5] = status_data->sensors[GYRZ];

    extra_analog_msg.layout.dim[2].label = "analog_inputs";
    extra_analog_msg.layout.dim[2].size = 4;
    extra_analog_msg.data[6] = status_data->sensors[ANA0];
    extra_analog_msg.data[7] = status_data->sensors[ANA1];
    extra_analog_msg.data[8] = status_data->sensors[ANA2];
    extra_analog_msg.data[9] = status_data->sensors[ANA3];

    if (extra_analog_inputs_publisher->trylock())
    {
      extra_analog_inputs_publisher->msg_ = extra_analog_msg;
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

void SR08::reinitialize_boards()
{
  // Reinitialize motors information
  sr_hand_lib->reinitialize_motors();
}

void SR08::get_board_id_and_can_bus(int board_id, int *can_bus, unsigned int *board_can_id)
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

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */
