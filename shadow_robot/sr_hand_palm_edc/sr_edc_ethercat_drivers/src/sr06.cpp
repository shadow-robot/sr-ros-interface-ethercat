/**
 * @file   sr06.cpp
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
 *         Ugo Cupcic <ugo@shadowrobot.com>, contact <contact@shadowrobot.com>
 * @date   Mon May 23 13:33:30 2011
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
 * @brief This is a ROS driver for Shadow Robot #6 EtherCAT Slave
 *
 *
 */


#include <sr_edc_ethercat_drivers/sr06.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>
#include <realtime_tools/realtime_publisher.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <std_msgs/Int16.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>
#include <bfd.h>

using namespace std;

#include <sr_edc_ethercat_drivers/types_for_external.h>
extern "C" {
  #include "external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
  #include "external/simplemotor-bootloader/bootloader.h"
}

#define ETHERCAT_STATUS_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND)

const unsigned short int SR06::device_pub_freq_const      = 1000;
const unsigned short int SR06::ros_pub_freq_const         = 1000;
const unsigned short int SR06::max_iter_const             = device_pub_freq_const / ros_pub_freq_const;
const unsigned int       SR06::nb_sensors_const           = ETHERCAT_STATUS_DATA_SIZE/2; //36;
const unsigned char      SR06::nb_publish_by_unpack_const = (nb_sensors_const % max_iter_const) ? (nb_sensors_const / max_iter_const) + 1 : (nb_sensors_const / max_iter_const);
const unsigned int       SR06::max_retry                  = 10;

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)


PLUGINLIB_REGISTER_CLASS(6, SR06, EthercatDevice);

#define check_for_pthread_mutex_init_error(x)	switch(x) \
						{ \
							case EAGAIN: \
                                                          ROS_ERROR("The system temporarily lacks the resources to create another mutex : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case EINVAL: \
								ROS_ERROR("The value specified as attribute is invalid for mutex init : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case ENOMEM: \
								ROS_ERROR("The process cannot allocate enough memory to create another mutex : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case 0: /* SUCCESS */ \
								break; \
							default: \
								ROS_ERROR("unknown error value, is this POSIX system ? : %s:%d", __FILE__, __LINE__); \
								exit(1); \
						}

#define unlock(x)	switch ( pthread_mutex_unlock(x) ) \
			{ \
				case EINVAL: \
					ROS_ERROR("The value specified as a mutex is invalid : %s:%d", __FILE__, __LINE__); \
					exit(1); \
					break; \
				case EPERM: \
					ROS_ERROR("The current thread does not hold a lock on the mutex : %s:%d", __FILE__, __LINE__); \
					exit(1); \
					break; \
			}

#define check_for_trylock_error(x)	if (x == EINVAL) \
					{ \
						ROS_ERROR("mutex error %s:%d", __FILE__, __LINE__); \
						exit(1); \
					}


/** \brief Constructor of the SR06 driver
 *
 *  This is the Constructor of the driver. it creates a bunch of real time publishers to publich the joints data
 *  initializes a few boolean values, a mutex and creates the Flashing service.
 */
//, com_(EthercatDirectCom(EtherCAT_DataLinkLayer::instance()))

SR06::SR06()
  : SR0X(),
    flashing(false),
    can_message_sent(true),
    can_packet_acked(true),
    zero_buffer_read(0)
{
  int res;
  check_for_pthread_mutex_init_error(res);
  counter_ = 0;

  std::vector<motor_updater::UpdateConfig> update_rate_configs_vector = read_update_rate_configs();
  motor_updater = boost::shared_ptr<motor_updater::MotorUpdater>(new motor_updater::MotorUpdater(update_rate_configs_vector));

  ROS_INFO("There are %d sensors", nb_sensors_const);

/*
  if (EC_PALM_EDC_COMMAND_PHY_BASE+ETHERCAT_COMMAND_DATA_SIZE > EC_PALM_EDC_CAN_BRIDGE_MASTER_OUT_BASE)
  ROS_ERROR("Not enough space for ETHERCAT_COMMAND_DATA\n");
  else
  ROS_ERROR("Enough space for ETHERCAT_COMMAND_DATA\n");

  if (EC_PALM_EDC_CAN_BRIDGE_MASTER_OUT_BASE+sizeof( ETHERCAT_CAN_BRIDGE_DATA) > EC_PALM_EDC_DATA_PHY_BASE)
  ROS_ERROR("Not enough space for EC_PALM_EDC_CAN_BRIDGE_MASTER_OUT_BASE\n");
  else
  ROS_ERROR("Enough space for EC_PALM_EDC_CAN_BRIDGE_MASTER_OUT_BASE\n");

  if (EC_PALM_EDC_DATA_PHY_BASE+ETHERCAT_STATUS_DATA_SIZE > EC_PALM_EDC_CAN_BRIDGE_MASTER_IN_BASE)
  ROS_ERROR("Not enough space for EC_PALM_EDC_DATA_PHY_BASE\n");
  else
  ROS_ERROR("Enough space for EC_PALM_EDC_DATA_PHY_BASE\n");
*/
  ROS_INFO(     "device_pub_freq_const = %d", device_pub_freq_const      );
  ROS_INFO(        "ros_pub_freq_const = %d", ros_pub_freq_const         );
  ROS_INFO(            "max_iter_const = %d", max_iter_const             );
  ROS_INFO(          "nb_sensors_const = %d", nb_sensors_const           );
  ROS_INFO("nb_publish_by_unpack_const = %d", nb_publish_by_unpack_const );

  res = pthread_mutex_init(&producing, NULL);
  check_for_pthread_mutex_init_error(res);

  serviceServer = nodehandle_.advertiseService("SimpleMotorFlasher", &SR06::simple_motor_flasher, this);
}

/** \brief Desctructor of the SR06 driver
 *
 *  This is the Destructor of the driver. it frees the FMMUs and SyncManagers which have been allocated during the construct.
 */
SR06::~SR06()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
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
 * If you need to have several commands like in this SR06 driver, put the sum of the size, same thing for the status.
 *
 */
void SR06::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
    SR0X::construct(sh, start_address);

    command_base_ = start_address;
    command_size_ = ETHERCAT_COMMAND_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE;

    start_address += ETHERCAT_COMMAND_DATA_SIZE;
    start_address += ETHERCAT_CAN_BRIDGE_DATA_SIZE;

    status_base_ = start_address;
    status_size_ = ETHERCAT_STATUS_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE;


    // ETHERCAT_COMMAND_DATA
    //
    // This is for data going TO the palm
    //
    ROS_ERROR("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_, command_size_, (int)ETHERCAT_COMMAND_DATA_ADDRESS);
    EC_FMMU *commandFMMU = new EC_FMMU( command_base_,
                                        command_size_,                                                  // Logical Start Address    (in ROS address space?)
                                        0x00,                                                           // Logical Start Bit
                                        0x07,                                                           // Logical End Bit
                                        ETHERCAT_COMMAND_DATA_ADDRESS,                                  // Physical Start Address   (in ET1200 address space?)
                                        0x00,                                                           // Physical Start Bit
                                        false,                                                          // Read Enable
                                        true,                                                           // Write Enable
                                        true                                                            // Channel Enable
                                       );




    // ETHERCAT_STATUS_DATA
    //
    // This is for data coming FROM the palm
    //
    ROS_ERROR("Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", status_base_, status_size_, (int)ETHERCAT_STATUS_DATA_ADDRESS);
    EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                        status_size_,
                                        0x00,
                                        0x07,
                                        ETHERCAT_STATUS_DATA_ADDRESS,
                                        0x00,
                                        true,
                                        false,
                                        true);



    EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

    (*fmmu)[0] = *commandFMMU;
    (*fmmu)[1] = *statusFMMU;

    sh->set_fmmu_config(fmmu);

    EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

    (*pd)[0] = EC_SyncMan(ETHERCAT_COMMAND_DATA_ADDRESS,             ETHERCAT_COMMAND_DATA_SIZE,    EC_QUEUED, EC_WRITTEN_FROM_MASTER);
    (*pd)[1] = EC_SyncMan(ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS,  ETHERCAT_CAN_BRIDGE_DATA_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
    (*pd)[2] = EC_SyncMan(ETHERCAT_STATUS_DATA_ADDRESS,              ETHERCAT_STATUS_DATA_SIZE,     EC_QUEUED);
    (*pd)[3] = EC_SyncMan(ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS,   ETHERCAT_CAN_BRIDGE_DATA_SIZE, EC_QUEUED);

    status_size_ = ETHERCAT_STATUS_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE;

    (*pd)[0].ChannelEnable = true;
    (*pd)[0].ALEventEnable = true;
    (*pd)[0].WriteEvent    = true;

    (*pd)[1].ChannelEnable = true;
    (*pd)[1].ALEventEnable = true;
    (*pd)[1].WriteEvent    = true;

    (*pd)[2].ChannelEnable = true;
    (*pd)[3].ChannelEnable = true;

    sh->set_pd_config(pd);

    ROS_ERROR("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);

    ROS_INFO("Finished constructing the SR06 driver");
}

/**
 *
 */
int SR06::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{

  int retval = SR0X::initialize(hw, allow_unprogrammed);

  if(retval != 0)
    return retval;

  //TODO: read this from config/EEProm?
  std::vector<shadow_joints::JointToSensor > joint_to_sensor_vect = read_joint_to_sensor_mapping();

  //initializing the hand library
  std::vector<std::string> joint_names_tmp;
  std::vector<int> motor_ids;
  std::vector<shadow_joints::JointToSensor > joints_to_sensors;
  std::vector<pr2_hardware_interface::Actuator*> actuators;

  ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM);

  for(unsigned int i=0; i< JOINTS_NUM; ++i)
  {
    joint_names_tmp.push_back(std::string(joint_names[i]));
    motor_ids.push_back(i);
    shadow_joints::JointToSensor tmp_jts = joint_to_sensor_vect[i];
    joints_to_sensors.push_back(tmp_jts);

    //initializing the actuators.
    pr2_hardware_interface::Actuator* actuator = new pr2_hardware_interface::Actuator(joint_names[i]);
    ROS_ERROR_STREAM("adding actuator: "<<joint_names[i]);
    actuators.push_back( actuator );

    if(hw)
    {
      if(!hw->addActuator(actuator) )
      {
        ROS_FATAL("An actuator of the name '%s' already exists.", actuator->name_.c_str());
        return -1;
      }
    }
  }
  sr_hand_lib = boost::shared_ptr<shadow_robot::SrHandLib>(new shadow_robot::SrHandLib(joint_names_tmp, motor_ids, joints_to_sensors, actuators));




  ROS_ERROR("ETHERCAT_STATUS_DATA_SIZE      = %4d bytes", (int)ETHERCAT_STATUS_DATA_SIZE);
  ROS_ERROR("ETHERCAT_COMMAND_DATA_SIZE     = %4d bytes", (int)ETHERCAT_COMMAND_DATA_SIZE);
  ROS_ERROR("ETHERCAT_CAN_BRIDGE_DATA_SIZE  = %4d bytes", (int)ETHERCAT_CAN_BRIDGE_DATA_SIZE);


//  com_ = EthercatDirectCom(EtherCAT_DataLinkLayer::instance());

  return retval;
}

/** \brief Erase the PIC18F Flash memory
 *
 *  This function fills the can_message_ struct with a CAN message
 *  which tells the bootloader of the PIC18F to erase its Flash memory
 */
void SR06::erase_flash(void)
{
  unsigned char cmd_sent;
  unsigned int wait_time;
  bool timedout;
  unsigned int timeout;
  int err;

  do {
    ROS_ERROR("Sending the ERASE FLASH command");
    // First we send the erase command
    cmd_sent = 0;
    while (! cmd_sent )
    {
      if ( !(err = pthread_mutex_trylock(&producing)) )
      {
        can_message_.message_length = 1;
        can_message_.can_bus = 1;
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
    while ( !can_packet_acked )
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
  } while (timedout);
}

/** \brief Function that reads back 8 bytes from PIC18F program memory
 *
 *  Flash memory is the program memory on the PIC18F
 *  This function is here to read back what we flashed into the PIC18F
 *  To check that there was no flashing transmission or writting error
 *  during the Flashing process.
 *  8 bytes will be read, from address (baddru << 16) + (baddrh << 8) + baddrl + offset
 *  This function will fill the can_message_ structure with the correct value
 *  to allow the packCommand() function to send the correct etherCAT message
 *  to the PIC32 which will then send the correct CAN message to the PIC18F
 *
 * @param offset The position of the 8 bytes we want to read, relative to the base address
 * @param baddrl least significant byte of the base address
 * @param baddrh middle byte of the base address
 * @param baddru upper byte of the base address
 *
 * @return Returns true if the command has timed out, false if the command was properly acknowledged
 */
bool SR06::read_flash(unsigned int offset, unsigned char baddrl, unsigned char baddrh, unsigned char baddru)
{
  unsigned int cmd_sent;
  int err;
  unsigned int wait_time;
  bool timedout;
  unsigned int timeout;
  cmd_sent = 0;
  while ( !cmd_sent )
  {
    if ( !(err = pthread_mutex_trylock(&producing)) )
    {
      ROS_ERROR("Sending READ data ... position : %d", pos);
      can_message_.can_bus = 1;
      can_message_.message_length = 3;
      can_message_.message_id = 0x0600 | (motor_being_flashed << 5) | READ_FLASH_COMMAND;
      can_message_.message_data[2] = baddru + ((offset + baddrl + (baddrh << 8)) >> 16);
      can_message_.message_data[1] = baddrh + ((offset + baddrl) >> 8); // User application start address is 0x4C0
      can_message_.message_data[0] = baddrl + offset;
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
  while ( !can_packet_acked )
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
 *  the message to be sent by the SR06::packCommand() function. And then repeat the process for another message, and so on.
 *
 *  This service will first read all the sections of the firmware using libbfd and find out the lowest and highest addresses containing code.
 *  Then it will allocate an array to contain the firmware's code. The size is (highest_addr - lowest_addr).
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
bool SR06::simple_motor_flasher(sr_edc_ethercat_drivers::SimpleMotorFlasher::Request &req, sr_edc_ethercat_drivers::SimpleMotorFlasher::Response &res)
{
  int err;
  unsigned char cmd_sent;
  bfd *fd;
  asection *s;
  //const char *section_name;
  unsigned int section_size = 0;
  unsigned int section_addr = 0;
  unsigned char addrl;
  unsigned char addrh;
  unsigned char addru;
  int nb_sections;
  unsigned int smallest_start_address = 0x7fff;
  unsigned int biggest_end_address = 0;
  unsigned int total_size = 0;
  int timeout, wait_time;
  bool timedout;

  motor_being_flashed = req.motor_id;
  binary_content = NULL;
  flashing = true;

  ROS_ERROR("Flashing the motor\n");

  bfd_init();

  fd = bfd_openr(req.firmware.c_str(), NULL);
  if (fd == NULL)
  {
    ROS_FATAL("error opening the file %s", req.firmware.c_str());
  }
/*	for (s = fd->sections ; s ; s = s->next)
	{*/
  if (!bfd_check_format (fd, bfd_object)) {
    if (bfd_get_error () != bfd_error_file_ambiguously_recognized) {
      ROS_FATAL("Incompatible format");
    }
  }


  ROS_ERROR("firmware %s's format is : %s.", req.firmware.c_str(), fd->xvec->name);

  ROS_ERROR("Sending dummy packet");
  cmd_sent = 0;
  while ( !cmd_sent )
  {
    if ( !(err = pthread_mutex_trylock(&producing)) )
    {
      can_message_.message_length = 0;
      can_message_.can_bus = 1;
      can_message_.message_id = 0;
      cmd_sent = 1;
      unlock(&producing);
    }
    else
    {
      check_for_trylock_error(err);
    }
  }
  wait_time = 0;
  timeout = 1;
  timedout = false;
  can_message_sent = false;
  can_packet_acked = false;
  while ( !can_packet_acked )
  {
    usleep(1000); // 1 ms
    wait_time++;
    if (wait_time > timeout) {
      timedout = true;
      break;
    }
  }

  ROS_ERROR("Sending magic CAN packet to put the motor in bootloading mode");
  cmd_sent = 0;
  while ( !cmd_sent )
  {
    if ( !(err = pthread_mutex_trylock(&producing)) )
    {
      can_message_.message_length = 8;
      can_message_.can_bus = 1;
      can_message_.message_id = 0x0400 | (req.motor_id << 5) | 0b1010;
      can_message_.message_data[0] = 0x55;
      can_message_.message_data[1] = 0xAA;
      can_message_.message_data[2] = 0x55;
      can_message_.message_data[3] = 0xAA;
      can_message_.message_data[4] = 0x55;
      can_message_.message_data[5] = 0xAA;
      can_message_.message_data[6] = 0x55;
      can_message_.message_data[7] = 0xAA;
      cmd_sent = 1;
      unlock(&producing);
    }
    else
    {
      check_for_trylock_error(err);
    }
  }
  wait_time = 0;
  timeout = 100;
  timedout = false;
  can_message_sent = false;
  can_packet_acked = false;
  while ( !can_packet_acked )
  {
    usleep(1000); // 1 ms
    wait_time++;
    if (wait_time > timeout) {
      timedout = true;
      break;
    }
  }

  if ( timedout ) {
    ROS_ERROR("First magic CAN packet timedout");
    ROS_ERROR("Sending another magic CAN packet to put the motor in bootloading mode");
    cmd_sent = 0;
    while ( !cmd_sent )
    {
      if ( !(err = pthread_mutex_trylock(&producing)) )
      {
        can_message_.message_length = 8;
        can_message_.can_bus = 1;
        can_message_.message_id = 0x0600 | (req.motor_id << 5) | 0b1010;
        can_message_.message_data[0] = 0x55;
        can_message_.message_data[1] = 0xAA;
        can_message_.message_data[2] = 0x55;
        can_message_.message_data[3] = 0xAA;
        can_message_.message_data[4] = 0x55;
        can_message_.message_data[5] = 0xAA;
        can_message_.message_data[6] = 0x55;
        can_message_.message_data[7] = 0xAA;
        cmd_sent = 1;
        unlock(&producing);
      }
      else
      {
        check_for_trylock_error(err);
      }
    }
    wait_time = 0;
    timeout = 100;
    timedout = false;
    can_message_sent = false;
    can_packet_acked = false;
    while ( !can_packet_acked )
    {
      usleep(1000); // 1 ms
      wait_time++;
      if (wait_time > timeout) {
        timedout = true;
        break;
      }
    }
    if (timedout)
    {
      ROS_FATAL("None of the magic packets were ACKed");
      ROS_BREAK();
    }

  }

  erase_flash();

  sleep(1);

  for (s = fd->sections ; s ; s = s->next)
  {
    if (bfd_get_section_flags (fd, s) & (SEC_LOAD))
    {
      if (bfd_section_lma (fd, s) == bfd_section_vma (fd, s))
      {
        section_addr = (unsigned int) bfd_section_lma (fd, s);
        if (section_addr >= 0x7fff)
          continue;
        nb_sections++;
        section_size = (unsigned int) bfd_section_size (fd, s);
        smallest_start_address = min(section_addr, smallest_start_address);
        biggest_end_address = max(biggest_end_address, section_addr + section_size);
      }
    }
  }
  total_size = biggest_end_address - smallest_start_address;
  binary_content = (bfd_byte *)malloc(total_size);
  if (binary_content == NULL)
    ROS_FATAL("Error allocating memory for binary_content");
  memset(binary_content, 0xFF, total_size);


  for (s = fd->sections ; s ; s = s->next)
  {
    if (bfd_get_section_flags (fd, s) & (SEC_LOAD))
    {
      if (bfd_section_lma (fd, s) == bfd_section_vma (fd, s))
      {
        section_addr = (unsigned int) bfd_section_lma (fd, s);
        if (section_addr >= 0x7fff)
          continue;
        section_size = (unsigned int) bfd_section_size (fd, s);
        bfd_get_section_contents(fd, s, binary_content + (section_addr - smallest_start_address), 0, section_size);
      }
      else
      {
        ROS_FATAL("something went wrong while parsing %s.", req.firmware.c_str());
      }
    }
    else
    {
      ROS_FATAL("something went wrong while parsing %s.", req.firmware.c_str());
    }
  }
  addrl = smallest_start_address & 0xff;
  addrh = (smallest_start_address & 0xff00) >> 8;
  addru = smallest_start_address >> 16;

  pos = 0;
  unsigned int packet = 0;
  ROS_ERROR("Sending the firmware data\n");
  while ( pos < ((total_size % 32) == 0 ? total_size : (total_size + 32 - (total_size % 32))) )
  {
    if ((pos % 32) == 0)
    {
    send_address:
      packet = 0;
      do {
        cmd_sent = 0;
        while (! cmd_sent )
        {
          if ( !(err = pthread_mutex_trylock(&producing)) )
          {
            can_message_.message_length = 3;
            can_message_.can_bus = 1;
            can_message_.message_id = 0x0600 | (req.motor_id << 5) | WRITE_FLASH_ADDRESS_COMMAND;
            can_message_.message_data[2] = addru + ((pos + addrl + (addrh << 8)) >> 16);
            can_message_.message_data[1] = addrh + ((pos + addrl) >> 8); // User application start address is 0x4C0
            can_message_.message_data[0] = addrl + pos;
            ROS_ERROR("Sending write address : 0x%02X%02X%02X", can_message_.message_data[2], can_message_.message_data[1], can_message_.message_data[0]);
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
        while ( !can_packet_acked )
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
          ROS_ERROR("WRITE ADDRESS timedout ");
      } while ( timedout );
    }
    cmd_sent = 0;
    while (! cmd_sent )
    {
      if ( !(err = pthread_mutex_trylock(&producing)) )
      {
        ROS_ERROR("Sending data ... position == %d", pos);
        can_message_.message_length = 8;
        can_message_.can_bus = 1;
        can_message_.message_id = 0x0600 | (req.motor_id << 5) | WRITE_FLASH_DATA_COMMAND;
        bzero(can_message_.message_data, 8);
        for (unsigned char j = 0 ; j < 8 ; ++j)
          can_message_.message_data[j] = (pos > total_size) ? 0xFF : *(binary_content + pos + j);
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
    while ( !can_packet_acked )
    {
      usleep(1000);
      if (wait_time > timeout)
      {
        timedout = true;
        break;
      }
      wait_time++;
    }
    if ( timedout )
    {
      ROS_ERROR("A WRITE data packet has been lost, resending the 32 bytes block !");
      pos -= packet*8;
      goto send_address;
    }
  }

//	close(fd); // We do not need the file anymore
  bfd_close(fd);

  // Now we have to read back the flash content
  pos = 0;
  unsigned int retry;
  while (pos < total_size)
  {
    retry = 0;
    do {
      timedout = read_flash(pos, addrl, addrh, addru);
      if ( ! timedout )
        pos += 8;
      retry++;
      if (retry > max_retry)
      {
        ROS_FATAL("Too much retry for READ back, try flashing again");
        return true;
      }
    } while ( timedout );
  }

  free(binary_content);
  ROS_ERROR("Sending the RESET command to PIC18F");
  // Then we send the RESET command to PIC18F
  cmd_sent = 0;
  while (! cmd_sent )
  {
    if ( !(err = pthread_mutex_trylock(&producing)) )
    {
      can_message_.message_length = 0;
      can_message_.can_bus = 1;
      can_message_.message_id = 0x0600 | (req.motor_id << 5) | RESET_COMMAND;
      cmd_sent = 1;
      unlock(&producing);
    }
    else
    {
      check_for_trylock_error(err);
    }
  }

  can_message_sent = false;
  can_packet_acked = false;
  while ( !can_packet_acked )
  {
    usleep(1);
  }

  flashing = false;

  ROS_ERROR("Flashing done");

  res.value = res.SUCCESS;
  return true;
}

/** \brief This function gives some diagnostics data
 *
 *  This function provides diagnostics data that can be displayed by
 *  the runtime_monitor node. We use the mutliDiagnostics as it publishes
 *  the diagnostics for each motors.
 */
void SR06::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer)
{
  diagnostic_updater::DiagnosticStatusWrapper &d(diagnostic_status_);

  stringstream name;
  name << "EtherCAT Device #" << setw(2) << setfill('0')
       << sh_->get_ring_position() << " (Product SIX)";
  d.name = name.str();
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

  this->ethercatDiagnostics(d,2);
  vec.push_back(d);

  shadow_joints::JointsMap::iterator joint_iter;
  for(joint_iter = sr_hand_lib->joints_map.begin(); joint_iter != sr_hand_lib->joints_map.end();
      ++joint_iter)
  {
    const pr2_hardware_interface::ActuatorState *state(&(joint_iter->second->motor->actuator)->state_);

    name.str("");
    name << "SRDMotor "<< joint_iter->first;
    d.name = name.str();

    //TODO check if motor is OK
    d.summary(d.OK, "OK");

    d.clear();
    d.addf("Measured Voltage", "%f", state->motor_voltage_);
    d.addf("Measured Current", "%f", state->last_measured_current_);

    d.addf("Measured Effort", "%f", state->last_measured_effort_);
    d.addf("Encoder Position", "%f", state->position_);

    vec.push_back(d);
  }
}



/** \brief packs the commands before sending them to the EtherCAT bus
 *
 *  This is one of the most important functions of this driver.
 *  This function is called each millisecond (1 kHz freq) by the EthercatHardware::update() function
 *  in the controlLoop() of the pr2_etherCAT node.
 *
 *  This function is called with a buffer as a parameter, the buffer provided is where we write the commands to send via EtherCAT.
 *
 *  We just cast the buffer to our structure type, fill the structure with our data, then add the structure size to the buffer address to shift into memory and access the second command.
 *  The buffer has been allocated with command_size_ bytes, which is the sum of the two command size, so we have to put the two commands one next to the other.
 *  In fact we access the buffer using this kind of code : \code
 *  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND  *command = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *)buffer;
 *  ETHERCAT_CAN_BRIDGE_DATA                       *message = (ETHERCAT_CAN_BRIDGE_DATA *)(buffer + ETHERCAT_COMMAND_DATA_SIZE);
 *  \endcode
 */
void SR06::packCommand(unsigned char *buffer, bool halt, bool reset)
{

  int res;

  SR0X::packCommand(buffer, halt, reset);

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND   *command = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND *)(buffer                             );
  ETHERCAT_CAN_BRIDGE_DATA	                  *message = (ETHERCAT_CAN_BRIDGE_DATA                      *)(buffer + ETHERCAT_COMMAND_DATA_SIZE);




  if ( !flashing )
  {
    command->EDC_command = EDC_COMMAND_SENSOR_DATA;
  }
  else
  {
    command->EDC_command = EDC_COMMAND_CAN_DIRECT_MODE;
  }

  // Currently, the only data we send to motors is torque demand.
  command->to_motor_data_type   = MOTOR_DEMAND_TORQUE;

  //alternate between even and uneven motors
  // and ask for the different informations.
  motor_updater->build_update_motor_command(command);

  if (flashing && !can_packet_acked && !can_message_sent)
  {
    if ( !(res = pthread_mutex_trylock(&producing)) )
    {
      ROS_DEBUG("We send a CAN message for flashing !");
      memcpy(message, &can_message_, sizeof(can_message_));
      can_message_sent = true;
      ROS_DEBUG("Sending : SID : 0x%04X ; bus : 0x%02X ; length : 0x%02X ; data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 	message->message_id,
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
      ROS_DEBUG("Mutex is locked, we don't send any CAN message !");
      check_for_trylock_error(res);
    }
  }
  else
  {
    message->can_bus        = 1;
    message->message_id     = 0x00;
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
bool SR06::can_data_is_ack(ETHERCAT_CAN_BRIDGE_DATA * packet)
{
  int i;

  if (packet->message_id == 0)
    return false;

  ROS_ERROR("ack sid : %04X", packet->message_id);

  if ( (packet->message_id & 0b0000011111111111) == (0x0600 | (motor_being_flashed << 5) | 0x10 | READ_FLASH_COMMAND))
    return ( !memcmp(packet->message_data, binary_content + pos, 8) );

  if (packet->message_length != can_message_.message_length)
    return false;
  ROS_ERROR("Length is OK");

  for (i = 0 ; i < packet->message_length ; ++i)
  {
    ROS_ERROR("packet sent, data[%d] : %02X ; ack, data[%d] : %02X", i, can_message_.message_data[i], i, packet->message_data[i]);
    if (packet->message_data[i] != can_message_.message_data[i])
      return false;
  }
  ROS_ERROR("Data is OK");

  if ( !(0x0010 & packet->message_id))
    return false;

  ROS_ERROR("This is an ACK");

  if ( (packet->message_id & 0b0000000111101111) != (can_message_.message_id & 0b0000000111101111) )
    return false;

  ROS_ERROR("SID is OK");

  ROS_ERROR("Everything is OK, this is our ACK !");
  return true;
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
 *  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *tbuffer = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *)(this_buffer + command_size_);
 *  ETHERCAT_CAN_BRIDGE_DATA *can_data = (ETHERCAT_CAN_BRIDGE_DATA *)(this_buffer + command_size_ + ETHERCAT_STATUS_DATA_SIZE);
 *  \endcode
 *
 * @param this_buffer The data just being received by EtherCAT
 * @param prev_buffer The previous data received by EtherCAT
 */
bool SR06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS  *status_data = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS *)(this_buffer + command_size_                             );
  ETHERCAT_CAN_BRIDGE_DATA                      *can_data    = (ETHERCAT_CAN_BRIDGE_DATA                     *)(this_buffer + command_size_ + ETHERCAT_STATUS_DATA_SIZE );
  int16u                                        *status_buffer = (int16u*)status_data;
  static unsigned int num_rxed_packets = 0;

  ++num_rxed_packets;
  if (status_data->EDC_command == EDC_COMMAND_INVALID)
  {
    //received empty message: the pic is not writing to its mailbox.
    ++zero_buffer_read;
    float percentage_packet_loss = 100.f * ((float)zero_buffer_read / (float)num_rxed_packets);
    ROS_ERROR("Reception error detected : %d errors out of %d rxed packets (%2.3f%%)", zero_buffer_read, num_rxed_packets, percentage_packet_loss);
    return false;
  }

  //ok the message was not empty
  //@TODO: how do we get the current actuator state?

  shadow_joints::JointsMap::iterator joint_iter;
  for(joint_iter = sr_hand_lib->joints_map.begin(); joint_iter != sr_hand_lib->joints_map.end();
      ++joint_iter)
  {
    pr2_hardware_interface::Actuator* actuator = joint_iter->second->motor->actuator;
    pr2_hardware_interface::ActuatorState* state(&actuator->state_);

    int i=joint_iter->second->motor->motor_id;

    state->is_enabled_ = 1;
    state->device_id_ = i;

    /////////////
    // Get the joint positions and compute the calibrate
    // values

    if(joint_iter->second->joint_to_sensor.calibrate_after_combining_sensors)
    {
      //first we combine the different sensors
      double raw_position = 0.0;
      BOOST_FOREACH(shadow_joints::PartialJointToSensor joint_to_sensor, joint_iter->second->joint_to_sensor.joint_to_sensor_vector)
        raw_position += static_cast<double>(status_data->sensors[joint_to_sensor.sensor_id])*joint_to_sensor.coeff;

      state->encoder_count_ = static_cast<int>(raw_position);

      //and now we calibrate
      //TODO: calibration!
      state->position_ = static_cast<double>(raw_position)/4000.0;
    }
    else
    {
      //we calibrate the different sensors and we combine the calibrated
      //values
      double calibrated_position = 0.0;
      BOOST_FOREACH(shadow_joints::PartialJointToSensor joint_to_sensor, joint_iter->second->joint_to_sensor.joint_to_sensor_vector)
      {
        //TODO: calibrate
        double raw_pos = static_cast<double>(status_data->sensors[joint_to_sensor.sensor_id]) / 4000.0;
        //combine
        calibrated_position += raw_pos * joint_to_sensor.coeff;
      }
      state->position_ = calibrated_position;

    }

              //
    ////////////


    //get the remaining information.
    // TODO: check if there was an error, using which_motor_data_had_errors mask
    bool read_motor_info = false;
    int index_motor = 0;
    if(status_data->which_motors == 0)
    {
      //We sampled the even motor numbers
      if( i%2 == 0)
      {
        read_motor_info = true;
        index_motor = i/2;
      }
    }
    else
    {
      //we sampled the uneven motor numbers
      if( i%2 == 1)
      {
        read_motor_info = true;
        index_motor = i/2 - 1;
      }
    }

    //ok now we read the info and add it to the actuator state
    if(read_motor_info)
    {
      switch(status_data->motor_data_type)
      {
      case MOTOR_DATA_VOLTAGE:
        //TODO: Hugo: how can I get the correct voltage from the motor (as a double)
        state->motor_voltage_ = (double)status_data->motor_data_packet[index_motor].misc;
        break;
      case MOTOR_DATA_CURRENT:
        state->last_measured_current_ = (double)status_data->motor_data_packet[index_motor].misc;
      default:
        break;
      }

      state->last_measured_effort_ = (double)status_data->motor_data_packet[index_motor].torque;
    }

  }

  if (flashing & !can_packet_acked)
  {
    if (can_data_is_ack(can_data))
    {
      can_packet_acked = true;
    }
  }

  ROS_DEBUG("Leaving UnpackState");

  return true;
}


std::vector<motor_updater::UpdateConfig> SR06::read_update_rate_configs()
{
  std::vector<motor_updater::UpdateConfig> update_rate_configs_vector;

  //TODO: This should be moved somewhere else. Not sure where yet.
  std::string base_topic = "motor_data_update_rate/";
  typedef std::pair<std::string, FROM_MOTOR_DATA_TYPE> ConfPair;
  std::vector<ConfPair> config;

  static const char* topics[13] = {"sgl", "sgr", "pwm", "flags", "current",
                                   "voltage", "temperature", "can_num_received",
                                   "can_num_transmitted", "svn_revision",
                                   "f_p", "i_d", "imax_deadband_sign"};

  static const FROM_MOTOR_DATA_TYPE data_types[13] = {MOTOR_DATA_SGL, MOTOR_DATA_SGR,
                                                      MOTOR_DATA_PWM, MOTOR_DATA_FLAGS,
                                                      MOTOR_DATA_CURRENT, MOTOR_DATA_VOLTAGE,
                                                      MOTOR_DATA_TEMPERATURE, MOTOR_DATA_CAN_NUM_RECEIVED,
                                                      MOTOR_DATA_CAN_NUM_TRANSMITTED, MOTOR_DATA_SVN_REVISION,
                                                      MOTOR_DATA_F_P, MOTOR_DATA_I_D,
                                                      MOTOR_DATA_IMAX_DEADBAND_SIGN};

  for(unsigned int i=0; i<13; ++i)
  {
    ConfPair tmp;
    tmp.first = base_topic + topics[i];
    tmp.second = data_types[i];
    config.push_back(tmp);
  }

  for(unsigned int i = 0; i < config.size(); ++i)
  {
    double rate;
    nodehandle_.getParam(config[i].first, rate);
    motor_updater::UpdateConfig config_tmp;

    config_tmp.when_to_update = rate;
    config_tmp.what_to_update = config[i].second;
    update_rate_configs_vector.push_back(config_tmp);
  }

  return update_rate_configs_vector;
}

std::vector<shadow_joints::JointToSensor> SR06::read_joint_to_sensor_mapping()
{
  std::vector<shadow_joints::JointToSensor> joint_to_sensor_vect;

  std::map<std::string, int> sensors_map;
  for(unsigned int i=0; i < SENSORS_NUM; ++i)
  {
    sensors_map[sensor_names[i] ] = i;
  }

  XmlRpc::XmlRpcValue joint_to_sensor_mapping;
  nodehandle_.getParam("joint_to_sensor_mapping", joint_to_sensor_mapping);
  ROS_ASSERT(joint_to_sensor_mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < joint_to_sensor_mapping.size(); ++i)
  {
    shadow_joints::JointToSensor tmp_vect;

    XmlRpc::XmlRpcValue map_one_joint = joint_to_sensor_mapping[i];

    //The parameter can either start by an array (sensor_name, coeff)
    // or by an integer to specify if we calibrate before combining
    // the different sensors
    int param_index = 0;
    //Check if the calibrate after combine int is set to 1
    if(map_one_joint[param_index].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      if(1 == static_cast<int>(map_one_joint[0]) )
        tmp_vect.calibrate_after_combining_sensors = true;
      else
        tmp_vect.calibrate_after_combining_sensors = false;

      param_index ++;
    }
    else //by default we calibrate before combining the sensors
      tmp_vect.calibrate_after_combining_sensors = false;

    ROS_ASSERT(map_one_joint.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = param_index; i < map_one_joint.size(); ++i)
    {
      ROS_ASSERT(map_one_joint[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      shadow_joints::PartialJointToSensor tmp_joint_to_sensor;

      ROS_ASSERT(map_one_joint[i][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      tmp_joint_to_sensor.sensor_id = sensors_map[ static_cast<std::string>(map_one_joint[i][0]) ];

      ROS_ASSERT(map_one_joint[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      tmp_joint_to_sensor.coeff = static_cast<double> (map_one_joint[i][1]);
      tmp_vect.joint_to_sensor_vector.push_back(tmp_joint_to_sensor);
    }
    joint_to_sensor_vect.push_back(tmp_vect);
  }

  return joint_to_sensor_vect;
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/