/**
 * @file   sr_edc.cpp
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
 *         Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
 * @date   Fri Mar 8 13:33:30 2013
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
 * @brief This is a parent class for the ROS drivers for any
 * Shadow Robot EtherCAT Dual CAN Slave.
 * It provides the tools to reprogram the Firmware of the microcontrollers
 * on the boards attached to the CAN busses of the Shadow EDC device
 * (like e.g. the motor boards, or the valve control boards),
 * assuming that they use the simplemotor-bootloader protocol implemented here.
 *
 *
 */


#include <sr_edc_ethercat_drivers/sr_edc.h>

#include <realtime_tools/realtime_publisher.h>

#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <std_msgs/Int16.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>
#include <bfd.h>

#include <sr_utilities/sr_math_utils.hpp>

using std::string;
using std::stringstream;
using std::vector;

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/simplemotor-bootloader/bootloader.h>
}

#include <boost/static_assert.hpp>

namespace is_edc_command_32_bits
{
// check is the EDC_COMMAND is 32bits on the computer
// if not, fails
  BOOST_STATIC_ASSERT(sizeof(EDC_COMMAND) == 4);
}  // namespace is_edc_command_32_bits

const unsigned int SrEdc::max_retry = 20;

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)


#define check_for_pthread_mutex_init_error(x)  switch (x)               \
  {                                                                     \
  case EAGAIN:                                                          \
    ROS_ERROR("The system temporarily lacks the resources to create another mutex : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
    break;                                                              \
  case EINVAL:                                                          \
    ROS_ERROR("The value specified as attribute is invalid for mutex init : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
    break;                                                              \
  case ENOMEM:                                                          \
    ROS_ERROR("The process cannot allocate enough memory to create another mutex : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
    break;                                                              \
  case 0: /* SUCCESS */                                                 \
    break;                                                              \
  default:                                                              \
    ROS_ERROR("unknown error value, is this POSIX system ? : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
  }

#define unlock(x)  switch ( pthread_mutex_unlock(x) )              \
  {                                                                     \
  case EINVAL:                                                          \
    ROS_ERROR("The value specified as a mutex is invalid : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
    break;                                                              \
  case EPERM:                                                           \
    ROS_ERROR("The current thread does not hold a lock on the mutex : %s:%d", __FILE__, __LINE__); \
    exit(1);                                                            \
    break;                                                              \
  }

#define check_for_trylock_error(x)  if (x == EINVAL)        \
  {                                                             \
    ROS_ERROR("mutex error %s:%d", __FILE__, __LINE__);         \
    exit(1);                                                    \
  }

/** \brief Constructor of the SrEdc driver
 *
 *  This is the Constructor of the driver. We
 *  initialize a few boolean values, a mutex
 *  and create the Bootloading service.
 */
SrEdc::SrEdc()
        : flashing(false),
          can_message_sent(true),
          can_packet_acked(true),
          can_bus_(0),
          counter_(0)
{
  int res = 0;
  check_for_pthread_mutex_init_error(res);

  res = pthread_mutex_init(&producing, NULL);
  check_for_pthread_mutex_init_error(res);
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
 * If you need to have several commands like in this SrEdc driver, put the sum of the size, same thing for the status.
 *
 */
void SrEdc::construct(EtherCAT_SlaveHandler *sh, int &start_address, unsigned int ethercat_command_data_size,
                      unsigned int ethercat_status_data_size, unsigned int ethercat_can_bridge_data_size,
                      unsigned int ethercat_command_data_address, unsigned int ethercat_status_data_address,
                      unsigned int ethercat_can_bridge_data_command_address,
                      unsigned int ethercat_can_bridge_data_status_address)
{
  sh_ = sh;

  // get the alias from the parameter server if it exists
  std::string path_to_alias, alias;
  path_to_alias = "/hand/mapping/" + boost::lexical_cast<std::string>(sh_->get_serial());
  ROS_INFO_STREAM("Trying to read mapping for: " << path_to_alias);
  if (ros::param::get(path_to_alias, alias))
  {
    device_id_ = alias;
  }
  else
  {
    // no alias found, using empty device_id_
    // Using the serial number as we do in ronex is probably a worse option here.
    device_id_ = "";
  }
  ros::NodeHandle nh_priv = ros::NodeHandle("~");
  bool use_ns = true;
  if (!nh_priv.getParam("use_ns", use_ns))
    ROS_INFO_STREAM("use_ns not set for " << nh_priv.getNamespace());

  if (use_ns)
  {
    nodehandle_ = ros::NodeHandle(device_id_);
    ROS_INFO_STREAM("Using namespace in sr_edc");
  }
  else
  {
    ROS_INFO_STREAM("Not using namespace in sr_edc");
    nodehandle_ = ros::NodeHandle();
  }
  nh_tilde_ = ros::NodeHandle(nh_priv, device_id_);

  serviceServer = nodehandle_.advertiseService("SimpleMotorFlasher", &SrEdc::simple_motor_flasher, this);

  // get the alias from the parameter server if it exists
  std::string path_to_prefix, prefix;
  path_to_prefix = "/hand/joint_prefix/" + boost::lexical_cast<std::string>(sh_->get_serial());
  ROS_INFO_STREAM("Trying to read joint_prefix for: " << path_to_prefix);
  if (ros::param::get(path_to_prefix, prefix))
  {
    device_joint_prefix_ = prefix;
  }
  else
  {
    // no prefix found, using empty prefix
    device_joint_prefix_ = "";
  }

  command_base_ = start_address;
  command_size_ = ethercat_command_data_size + ethercat_can_bridge_data_size;

  start_address += command_size_;

  status_base_ = start_address;
  status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

  start_address += status_size_;

  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the palm
  //
  ROS_INFO("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_,
           command_size_,
           static_cast<int> (ethercat_command_data_address));
  EC_FMMU *commandFMMU = new EC_FMMU(command_base_,  // Logical Start Address    (in ROS address space?)
                                     command_size_,
                                     0x00,  // Logical Start Bit
                                     0x07,  // Logical End Bit
                                     ethercat_command_data_address,  // Physical Start Address(in ET1200 address space?)
                                     0x00,  // Physical Start Bit
                                     false,  // Read Enable
                                     true,  // Write Enable
                                     true);  // Channel Enable




  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the palm
  //
  ROS_INFO("Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", status_base_,
           status_size_,
           static_cast<int> (ethercat_status_data_address));
  EC_FMMU *statusFMMU = new EC_FMMU(status_base_,
                                    status_size_,
                                    0x00,
                                    0x07,
                                    ethercat_status_data_address,
                                    0x00,
                                    true,
                                    false,
                                    true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

  (*pd)[0] = EC_SyncMan(ethercat_command_data_address, ethercat_command_data_size, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(ethercat_can_bridge_data_command_address, ethercat_can_bridge_data_size, EC_QUEUED,
                        EC_WRITTEN_FROM_MASTER);
  (*pd)[2] = EC_SyncMan(ethercat_status_data_address, ethercat_status_data_size, EC_QUEUED);
  (*pd)[3] = EC_SyncMan(ethercat_can_bridge_data_status_address, ethercat_can_bridge_data_size, EC_QUEUED);

  status_size_ = ethercat_status_data_size + ethercat_can_bridge_data_size;

  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent = true;

  (*pd)[1].ChannelEnable = true;
  (*pd)[1].ALEventEnable = true;
  (*pd)[1].WriteEvent = true;

  (*pd)[2].ChannelEnable = true;
  (*pd)[3].ChannelEnable = true;

  sh->set_pd_config(pd);

  ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);
}

/** \brief Erase the PIC18F Flash memory
 *
 *  This function fills the can_message_ struct with a CAN message
 *  which tells the bootloader of the PIC18F to erase its Flash memory
 */
void SrEdc::erase_flash(void)
{
  unsigned char cmd_sent;
  unsigned int wait_time;
  bool timedout = true;
  unsigned int timeout;
  int err;

  while (timedout)
  {
    ROS_INFO("Erasing FLASH");
    // First we send the erase command
    cmd_sent = 0;
    while (!cmd_sent)
    {
      if (!(err = pthread_mutex_trylock(&producing)))
      {
        can_message_.message_length = 1;
        can_message_.can_bus = can_bus_;
        can_message_.message_id = 0x0600 | (motor_being_flashed << 5) | ERASE_FLASH_COMMAND;
        cmd_sent = 1;
        unlock(&producing);
      }
      else
      {
        check_for_trylock_error(err);
      }
    }
    wait_time = 0;
    timeout = 3000;
    can_message_sent = false;
    can_packet_acked = false;
    timedout = false;
    while (!can_packet_acked)
    {
      usleep(1000);
      if (wait_time > timeout)
      {
        timedout = true;
        break;
      }
      wait_time++;
    }

    if (timedout)
    {
      ROS_ERROR("ERASE command timedout, resending it !");
    }
  };
}

/** \brief Function that reads back 8 bytes from PIC18F program memory
 *
 *  Flash memory is the program memory on the PIC18F
 *  This function is here to read back what we flashed into the PIC18F
 *  To check that there was no flashing transmission or writting error
 *  during the Flashing process.
 *  8 bytes will be read, from address baddr + offset
 *  This function will fill the can_message_ structure with the correct value
 *  to allow the packCommand() function to send the correct etherCAT message
 *  to the PIC32 which will then send the correct CAN message to the PIC18F
 *
 * @param offset The position of the 8 bytes we want to read, relative to the base address
 * @param baddr the base address
 *
 * @return Returns true if the command has timed out, false if the command was properly acknowledged
 */
bool SrEdc::read_flash(unsigned int offset, unsigned int baddr)
{
  unsigned int cmd_sent;
  int err;
  unsigned int wait_time;
  bool timedout;
  unsigned int timeout;
  cmd_sent = 0;
  while (!cmd_sent)
  {
    if (!(err = pthread_mutex_trylock(&producing)))
    {
      ROS_DEBUG("Sending READ data ... position : %03x", pos);
      can_message_.can_bus = can_bus_;
      can_message_.message_length = 3;
      can_message_.message_id = 0x0600 | (motor_being_flashed << 5) | READ_FLASH_COMMAND;
      can_message_.message_data[2] = (offset + baddr) >> 16;
      can_message_.message_data[1] = (offset + baddr) >> 8;  // User application start address is 0x4C0
      can_message_.message_data[0] = offset + baddr;
      cmd_sent = 1;
      unlock(&producing);
    }
    else
    {
      check_for_trylock_error(err);
    }
  }
  timedout = false;
  wait_time = 0;
  timeout = 100;
  can_message_sent = false;
  can_packet_acked = false;
  while (!can_packet_acked)
  {
    usleep(1000);
    if (wait_time > timeout)
    {
      timedout = true;
      break;
    }
    wait_time++;
  }
  return timedout;
}

/** \brief ROS Service that flashes a new firmware into a SimpleMotor board
 *
 *  This function is a ROS Service, aimed at flashing a new firmware into the
 *  PIC18F of a SimpleMotor board through a CAN bootloader protocol.
 *
 *  The CAN bootloader allows for several commands to be executed : read_flash, erase_flash, write_flash, reboot, read_version
 *
 *  This service will fill a can_message_ structure and then switch a few boolean values to "false" in order to trigger
 *  the message to be sent by the SRXX::packCommand() function. And then repeat the process for another message, and so on.
 *
 *  This service will first read all the sections of the firmware using libbfd and find out the lowest and highest addresses containing code.
 *  Then it will allocate an array to contain the firmware's code. The size is (highest_addr - lowest_addr).
 *  The bfd library provides functions to manage object files more easily. To better understand some of the concepts used below,
 *  the following link can be useful:
 *  http://www.delorie.com/gnu/docs/binutils/ld_7.html
 *  The use of the following commands can also help to understand the structure of the object file containing the firmware
 *  \code objdump -x simplemotor.hex \endcode
 *  \code objdump -s simplemotor.hex \endcode
 *
 *  - Then it will send a MAGIC PACKET command to the PIC18F which will make it reboot in bootloader mode (regardless of whether it was already in
 *  bootloader mode or whether it was running the SimpleMotor code)
 *  - Then it will send an ERASE_FLASH command to the PIC18F.
 *  - Then it will send a WRITE_FLASH_ADDRESS_COMMAND to tell the PIC18F
 *  where we wanna write, and then 4 WRITE_FLASH_DATA commands (we write by blocks of 32 bytes). This process is repeated untill we've written
 *  all the firmware code, padding with 0x00 bytes in the end if the size is not a multiple of 32 bytes.
 *  The process starts at address (lowest_addr) and ends at (hiest_addr) + a few padding bytes if necessary.
 *
 *  You can call this service using this command :
 *
 *  \code rosservice call SimpleMotorFlasher "/home/hand/simplemotor.hex" 8 \endcode
 *
 *  This will flash the "simplemotor.hex" firmware to the motor 8
 *
 *  @param req The Request, contains the ID of the motor we want to flash via req.motor_id, and the path of the firmware to flash in req.firmware
 *  @param res The Response, it is always SUCCESS for now.
 *
 *  @return This returns always true, the real return value is in the res parameter
 */
bool SrEdc::simple_motor_flasher(sr_robot_msgs::SimpleMotorFlasher::Request &req,
                                 sr_robot_msgs::SimpleMotorFlasher::Response &res)
{
  bfd *fd;
  unsigned int base_addr;
  unsigned int smallest_start_address = 0x7fff;
  unsigned int biggest_end_address = 0;
  unsigned int total_size = 0;
  bool timedout = true;

  get_board_id_and_can_bus(req.motor_id, &can_bus_, &motor_being_flashed);

  binary_content = NULL;
  flashing = true;

  ROS_INFO("Flashing the motor");

  // Initialize the bfd library: "This routine must be called before any other BFD
  // function to initialize magical internal data structures."
  bfd_init();

  // Open the requested firmware object file
  fd = bfd_openr(req.firmware.c_str(), NULL);
  if (fd == NULL)
  {
    ROS_ERROR("error opening the file %s", get_filename(req.firmware).c_str());
    res.value = res.FAIL;
    flashing = false;
    return false;
  }

  // Check that bfd recognises the file as a correctly formatted object file
  if (!bfd_check_format(fd, bfd_object))
  {
    if (bfd_get_error() != bfd_error_file_ambiguously_recognized)
    {
      ROS_ERROR("Incompatible format");
      res.value = res.FAIL;
      flashing = false;
      return false;
    }
  }

  ROS_INFO("firmware %s's format is : %s.", get_filename(req.firmware).c_str(), fd->xvec->name);

  // @todo Check if it's necessary to send this dummy packet before the magic packet
  ROS_DEBUG("Sending dummy packet");
  send_CAN_msg(can_bus_, 0, 0, NULL, 1, &timedout);

  ROS_INFO_STREAM("Switching motor " << motor_being_flashed << " on CAN bus " << can_bus_ << " into bootloader mode");
  // Send the magic packet that will force the microcontroller to go into bootloader mode
  int8u magic_packet[] = {0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA};
  send_CAN_msg(can_bus_, 0x0600 | (motor_being_flashed << 5) | 0b1010, 8, magic_packet, 100, &timedout);

  // Send a second magic packet if the first one wasn't acknowledged
  if (timedout)
  {
    ROS_WARN("First magic CAN packet timedout");
    ROS_WARN("Sending another magic CAN packet to put the motor in bootloading mode");
    send_CAN_msg(can_bus_, 0x0600 | (motor_being_flashed << 5) | 0b1010, 8, magic_packet, 100, &timedout);

    if (timedout)
    {
      ROS_ERROR("None of the magic packets were ACKed, didn't bootload the motor.");
      res.value = res.FAIL;
      flashing = false;
      return false;
    }
  }

  // Erase the PIC18 microcontroller flash memory
  erase_flash();

  sleep(1);

  // Look for the start and end address of every section in the hex file,
  // to detect the lowest and highest address of the data we need to write in the PIC's flash.
  find_address_range(fd, &smallest_start_address, &biggest_end_address);

  // Calculate the size of the chunk of data to be flashed
  total_size = biggest_end_address - smallest_start_address;
  base_addr = smallest_start_address;

  // Allocate the memory space to store the data to be flashed
  // This could be done with new bfd_byte[total_size+8] and delete() instead of malloc()
  // and free() but will stay this way for the moment
  binary_content = reinterpret_cast<bfd_byte *>(malloc(total_size + 8));
  if (binary_content == NULL)
  {
    ROS_ERROR("Error allocating memory for binary_content");
    res.value = res.FAIL;
    flashing = false;
    return false;
  }

  // Set all the bytes in the binary_content to 0xFF initially (i.e. before reading the content from the hex file)
  // This way we make sure that any byte in the region between smallest_start_address and biggest_end_address
  // that is not included in any section of the hex file, will be written with a 0xFF value,
  // which is the default in the PIC
  memset(binary_content, 0xFF, total_size + 8);

  // The content of the firmware is read from the .hex file pointed by fd, to a memory region pointed by binary_content
  if (!read_content_from_object_file(fd, binary_content, base_addr))
  {
    ROS_ERROR("something went wrong while parsing %s.", get_filename(req.firmware).c_str());
    res.value = res.FAIL;
    free(binary_content);
    flashing = false;
    return false;
  }

  // We do not need the file anymore
  bfd_close(fd);

  // The firmware is actually written to the flash memory of the PIC18
  if (!write_flash_data(base_addr, total_size))
  {
    res.value = res.FAIL;
    free(binary_content);
    flashing = false;
    return false;
  }


  ROS_INFO("Verifying");
  // Now we have to read back the flash content
  if (!read_back_and_check_flash(base_addr, total_size))
  {
    res.value = res.FAIL;
    free(binary_content);
    flashing = false;
    return false;
  }


  free(binary_content);

  ROS_INFO("Resetting microcontroller.");
  // Then we send the RESET command to PIC18F
  timedout = true;
  while (timedout)
  {
    send_CAN_msg(can_bus_, 0x0600 | (motor_being_flashed << 5) | RESET_COMMAND, 0, NULL, 1000, &timedout);
  };

  flashing = false;

  ROS_INFO("Flashing done");

  res.value = res.SUCCESS;

  // Reinitialize motor boards or valve controller boards information
  reinitialize_boards();

  return true;
}

void SrEdc::build_CAN_message(ETHERCAT_CAN_BRIDGE_DATA *message)
{
  int res;

  if (flashing && !can_packet_acked && !can_message_sent)
  {
    if (!(res = pthread_mutex_trylock(&producing)))
    {
      ROS_DEBUG_STREAM("Ethercat bridge data size: " << ETHERCAT_CAN_BRIDGE_DATA_SIZE);

      ROS_DEBUG("We're sending a CAN message for flashing.");
      memcpy(message, &can_message_, sizeof(can_message_));
      can_message_sent = true;

      ROS_DEBUG(
              "Sending : SID : 0x%04X ; bus : 0x%02X ; length : 0x%02X ;"
                      " data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
              message->message_id,
              message->can_bus,
              message->message_length,
              message->message_data[0],
              message->message_data[1],
              message->message_data[2],
              message->message_data[3],
              message->message_data[4],
              message->message_data[5],
              message->message_data[6],
              message->message_data[7]);

      unlock(&producing);
    }
    else
    {
      ROS_ERROR("Mutex is locked, we don't send any CAN message !");
      check_for_trylock_error(res);
    }
  }
  else
  {
    message->can_bus = can_bus_;
    message->message_id = 0x00;
    message->message_length = 0;
  }
}

/** \brief This function checks if the can packet in the unpackState() this_buffer is an ACK
 *
 *  This function checks several things on the can packet in this_buffer, it compares it with the
 *  can_message_ private member in several ways (SID, length, data) to check if it's an ACK.
 *
 *  @param packet The packet from this_buffer of unpackState() that we want to check if it's an ACK
 *  @return Returns true if packet is an ACK of can_message_ packet.
 */
bool SrEdc::can_data_is_ack(ETHERCAT_CAN_BRIDGE_DATA *packet)
{
  int i;

  if (packet->message_id == 0)
  {
    ROS_DEBUG("ID is zero");
    return false;
  }

  ROS_DEBUG("ack sid : %04X", packet->message_id);

  // Is this a reply to a READ request?
  if ((packet->message_id & 0b0000011111111111) == (0x0600 | (motor_being_flashed << 5) | 0x10 | READ_FLASH_COMMAND))
  {
    ROS_DEBUG("READ reply  %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", packet->message_data[0],
              packet->message_data[1],
              packet->message_data[2],
              packet->message_data[3],
              packet->message_data[4],
              packet->message_data[5],
              packet->message_data[6],
              packet->message_data[7]);
    ROS_DEBUG("Should be   %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", binary_content[pos + 0],
              binary_content[pos + 1],
              binary_content[pos + 2],
              binary_content[pos + 3],
              binary_content[pos + 4],
              binary_content[pos + 5],
              binary_content[pos + 6],
              binary_content[pos + 7]);

    if (!memcmp(packet->message_data, binary_content + pos, 8))
    {
      ROS_DEBUG("data is good");
      return true;
    }
    else
    {
      ROS_DEBUG("data is bad");
      return false;
    }
  }

  if (packet->message_length != can_message_.message_length)
  {
    ROS_DEBUG("Length is bad: %d", packet->message_length);
    return false;
  }

  ROS_DEBUG("Length is OK");

  for (i = 0; i < packet->message_length; ++i)
  {
    ROS_DEBUG("packet sent, data[%d] : %02X ; ack, data[%d] : %02X", i, can_message_.message_data[i], i,
              packet->message_data[i]);
    if (packet->message_data[i] != can_message_.message_data[i])
    {
      return false;
    }
  }
  ROS_DEBUG("Data is OK");

  if (!(0x0010 & packet->message_id))
  {
    return false;
  }

  ROS_DEBUG("This is an ACK");

  if ((packet->message_id & 0b0000000111101111) != (can_message_.message_id & 0b0000000111101111))
  {
    ROS_WARN_STREAM("Bad packet id: " << packet->message_id);
    return false;
  }

  ROS_DEBUG("SID is OK");

  ROS_DEBUG("Everything is OK, this is our ACK !");
  return true;
}

void SrEdc::send_CAN_msg(int8u can_bus, int16u msg_id, int8u msg_length, int8u msg_data[], int timeout, bool *timedout)
{
  int err;
  unsigned char cmd_sent;
  int wait_time;

  cmd_sent = 0;
  while (!cmd_sent)
  {
    if (!(err = pthread_mutex_trylock(&producing)))
    {
      can_message_.message_length = msg_length;
      can_message_.can_bus = can_bus;
      can_message_.message_id = msg_id;

      if (msg_data != NULL)
      {
        for (unsigned int i = 0; i < msg_length; i++)
        {
          can_message_.message_data[i] = msg_data[i];
        }
      }

      cmd_sent = 1;
      unlock(&producing);
    }
    else
    {
      check_for_trylock_error(err);
    }
  }
  wait_time = 0;
  *timedout = false;
  can_message_sent = false;
  can_packet_acked = false;
  while (!can_packet_acked)
  {
    usleep(1000);  // 1 ms
    wait_time++;
    if (wait_time > timeout)
    {
      *timedout = true;
      break;
    }
  }
}

bool SrEdc::read_back_and_check_flash(unsigned int baddr, unsigned int total_size)
{
  // The actual comparison between the content read from the flash and the content read from
  // the hex file is carried out in the can_data_is_ack() function.
  // read_flash(...) will return timedout = true if the 8 byte content read from the flash doesn't
  // match the 8 bytes from the hex file
  // BE CAREFUL with the pos "global" field, because it's being used inside can_data_is_ack()
  // function to check if the response of the READ_FLASH_COMMAND is correct
  pos = 0;
  unsigned int retry;
  while (pos < total_size)
  {
    bool timedout = true;

    retry = 0;
    while (timedout)
    {
      timedout = read_flash(pos, baddr);
      if (!timedout)
      {
        pos += 8;
      }
      retry++;
      if (retry > max_retry)
      {
        ROS_ERROR("Too much retry for READ back, try flashing again");
        return false;
      }
    };
  }
  return true;
}

void SrEdc::find_address_range(bfd *fd, unsigned int *smallest_start_address, unsigned int *biggest_end_address)
{
  asection *s;
  unsigned int section_size = 0;
  unsigned int section_addr = 0;

  // Look for the start and end address of every section in the hex file,
  // to detect the lowest and highest address of the data we need to write in the PIC's flash.
  // The sections starting at an address higher than 0x7fff will be ignored as they are not proper
  // "code memory" firmware
  // (they can contain the CONFIG bits of the microcontroller, which we don't want to write here)
  // To understand the structure (sections) of the object file containing the firmware (usually a .hex) the following
  // commands can be useful:
  //  \code objdump -x simplemotor.hex \endcode
  //  \code objdump -s simplemotor.hex \endcode
  for (s = fd->sections; s; s = s->next)
  {
    // Only the sections with the LOAD flag on will be considered
    if (bfd_get_section_flags(fd, s) & (SEC_LOAD))
    {
      // Only the sections with the same VMA (virtual memory address) and LMA (load MA) will be considered
      // http://www.delorie.com/gnu/docs/binutils/ld_7.html
      if (bfd_section_lma(fd, s) == bfd_section_vma(fd, s))
      {
        section_addr = (unsigned int) bfd_section_lma(fd, s);
        if (section_addr >= 0x7fff)
        {
          continue;
        }
        section_size = (unsigned int) bfd_section_size(fd, s);
        *smallest_start_address = std::min(section_addr, *smallest_start_address);
        *biggest_end_address = std::max(*biggest_end_address, section_addr + section_size);
      }
    }
  }
}

bool SrEdc::read_content_from_object_file(bfd *fd, bfd_byte *content, unsigned int base_addr)
{
  asection *s;
  unsigned int section_size = 0;
  unsigned int section_addr = 0;

  for (s = fd->sections; s; s = s->next)
  {
    // Only the sections with the LOAD flag on will be considered
    if (bfd_get_section_flags(fd, s) & (SEC_LOAD))
    {
      // Only the sections with the same VMA (virtual memory address) and LMA (load MA) will be considered
      // http://www.delorie.com/gnu/docs/binutils/ld_7.html
      if (bfd_section_lma(fd, s) == bfd_section_vma(fd, s))
      {
        section_addr = (unsigned int) bfd_section_lma(fd, s);
        // The sections starting at an address higher than 0x7fff will be ignored as they are
        // not proper "code memory" firmware
        // (they can contain the CONFIG bits of the microcontroller, which we don't want to write here)
        if (section_addr >= 0x7fff)
        {
          continue;
        }
        section_size = (unsigned int) bfd_section_size(fd, s);
        bfd_get_section_contents(fd, s, content + (section_addr - base_addr), 0, section_size);
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}

bool SrEdc::write_flash_data(unsigned int base_addr, unsigned int total_size)
{
  int err;
  unsigned char cmd_sent;
  int wait_time, timeout;

  pos = 0;
  unsigned int packet = 0;
  ROS_INFO("Sending the firmware data");
  while (pos < ((total_size % 32) == 0 ? total_size : (total_size + 32 - (total_size % 32))))
  {
    bool timedout = true;

    // For every WRITE_FLASH_ADDRESS_COMMAND we write 32 bytes of data to flash
    // and this is done by sending 4 WRITE_FLASH_DATA_COMMAND packets, every one containing
    // 8 bytes of data to be written
    if ((pos % 32) == 0)
    {
      packet = 0;
      while (timedout)
      {
        cmd_sent = 0;
        while (!cmd_sent)
        {
          if (!(err = pthread_mutex_trylock(&producing)))
          {
            can_message_.message_length = 3;
            can_message_.can_bus = can_bus_;
            can_message_.message_id = 0x0600 | (motor_being_flashed << 5) | WRITE_FLASH_ADDRESS_COMMAND;
            can_message_.message_data[2] = (base_addr + pos) >> 16;
            can_message_.message_data[1] = (base_addr + pos) >> 8;  // User application start address is 0x4C0
            can_message_.message_data[0] = base_addr + pos;
            ROS_DEBUG("Sending write address to motor %d : 0x%02X%02X%02X", motor_being_flashed,
                      can_message_.message_data[2], can_message_.message_data[1], can_message_.message_data[0]);
            cmd_sent = 1;
            unlock(&producing);
          }
          else
          {
            check_for_trylock_error(err);
          }
        }
        wait_time = 0;
        timedout = false;
        timeout = 100;
        can_message_sent = false;
        can_packet_acked = false;
        while (!can_packet_acked)
        {
          usleep(1000);
          if (wait_time > timeout)
          {
            timedout = true;
            break;
          }
          wait_time++;
        }
        if (timedout)
        {
          ROS_ERROR("WRITE ADDRESS timedout ");
        }
      };
    }
    cmd_sent = 0;
    while (!cmd_sent)
    {
      if (!(err = pthread_mutex_trylock(&producing)))
      {
        ROS_DEBUG("Sending data ... position == %d", pos);
        can_message_.message_length = 8;
        can_message_.can_bus = can_bus_;
        can_message_.message_id = 0x0600 | (motor_being_flashed << 5) | WRITE_FLASH_DATA_COMMAND;
        bzero(can_message_.message_data, 8);
        for (unsigned char j = 0; j < 8; ++j)
        {
          can_message_.message_data[j] = (pos > total_size) ? 0xFF : *(binary_content + pos + j);
        }

        pos += 8;
        cmd_sent = 1;
        unlock(&producing);
      }
      else
      {
        check_for_trylock_error(err);
      }
    }
    packet++;
    timedout = false;
    wait_time = 0;
    timeout = 100;
    can_message_sent = false;
    can_packet_acked = false;
    while (!can_packet_acked)
    {
      usleep(1000);
      if (wait_time > timeout)
      {
        timedout = true;
        break;
      }
      wait_time++;
    }
    if (timedout)
    {
      ROS_ERROR("A WRITE data packet has been lost at pos=%u, resending the 32 bytes block at pos=%u  !", pos,
                (pos - packet * 8));
      pos -= packet * 8;
    }
  }
  return true;
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */
