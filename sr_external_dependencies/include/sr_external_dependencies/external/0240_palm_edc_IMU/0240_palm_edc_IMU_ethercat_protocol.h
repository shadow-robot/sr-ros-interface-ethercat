/* 
* Copyright 2010, 2023-2024 Shadow Robot Company Ltd.
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
  * @file 0240_palm_edc_IMU_ethercat_protocol.h
  *
  * @brief EtherCat protocol for devices with Product ID 9
  *
  *  The Ethercat "Command" packet refers to data sent from the ROS host to the Node on the robot
  *  The Ethercat "Status" packet refers to data sent from Node on the robot the to the ROS host.
  */

#ifndef PALM_EDC_0240_ETHERCAT_PROTOCOL_H_INCLUDED
#define PALM_EDC_0240_ETHERCAT_PROTOCOL_H_INCLUDED

#include "../0220_palm_edc/0220_palm_edc_ethercat_protocol.h"

//! Data structure sent from the Palm to the host (Status).
//! Data is written to the EtherCAT bus sequencially, in the same order defined below.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< This tells us the contents of the data below.
                                                                    //!< This value should be identical to the EDC_command
                                                                    //!< value which arrived from the host in the previous
                                                                    //!< EtherCAT packet

    int16u                      sensors[SENSORS_NUM_0220+1];        //!< Joint sensors data

    FROM_MOTOR_DATA_TYPE        motor_data_type;                    //!< Which data does motor[] contain?
                                                                    //!< This value should agree with the previous value
                                                                    //!< in ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND
    int16s                      which_motors;                       //!< 0: Even motor numbers.  1: Odd motor numbers
                                                                    //!< This value should agree with the previous value
                                                                    //!< in ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND

    int32u                      which_motor_data_arrived;           //!< Bit N set when motor CAN message arrives. Ideally, bits 0..19 get set
    int32u                      which_motor_data_had_errors;        //!< Bit N set when motor sends bad CAN message Ideally, no bits get set.

    MOTOR_DATA_PACKET           motor_data_packet[10];              //!< Data for 10 motors only. (Even ones or Odd ones)

    int32u                      tactile_data_type;                  //!< Identifies the tactile data type
    int16u                      tactile_data_valid;                 //!< Bit 0: FF. Bit 4: TH.
    TACTILE_SENSOR_STATUS_v1    tactile[5];                         //!< Tactile sensors data

    int16u                      idle_time_us;                       //!< The idle time from when the palm has finished dealing with one EtherCAT
                                                                    //!< packet, and the next packet arriving. Ideally, this number should be more than 50.

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS;


//! Data structure sent from the host to the Palm (Command).
//! Data is written to the EtherCAT bus sequencially, in the same order defined below.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< What type of data should the palm send back in the next packet?

    FROM_MOTOR_DATA_TYPE        from_motor_data_type;               //!< Which data does the host want from the motors?
    int16s                      which_motors;                       //!< Which motors does the host want to read?
                                                                    //!< 0: Even motor numbers.  1: Odd motor numbers

    TO_MOTOR_DATA_TYPE          to_motor_data_type;
    int16s                      motor_data[NUM_MOTORS];             //!< Data to send to motors. Typically torque/PWM demands, or configs.

    int32u                      tactile_data_type;                  //!< Request for specific tactile data
    IMU_COMMAND_TYPE            imu_command;                        //!< Command to configure the IMU

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND;

//! EtherCAT protocol packet/header sizes
#define PALM_0240_ETHERCAT_COMMAND_HEADER_SIZE  (sizeof(EDC_COMMAND) + sizeof(FROM_MOTOR_DATA_TYPE) + sizeof(int16s))

#define PALM_0240_ETHERCAT_STATUS_DATA_SIZE       sizeof(ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS)
#define PALM_0240_ETHERCAT_COMMAND_DATA_SIZE      sizeof(ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND)

//! Ethercat Command and Status packets "agreed" sizes.
//! They are use by the host and clients to assert if the incoming packets are the correct size.
#define ETHERCAT_STATUS_0240_AGREED_SIZE     232  //!< This is the size of the Status  EtherCAT packet (Status + CAN packet)
#define ETHERCAT_COMMAND_0240_AGREED_SIZE    74   //!< This is the size of the Command EtherCAT packet (Command + CAN packet)



//! | ETHERCAT_COMMAND_DATA | ETHERCAT_CAN_BRIDGE_DATA_COMMAND | ETHERCAT_STATUS_DATA | ETHERCAT_CAN_BRIDGE_DATA_STATUS |
//! |                       |                                  |                      |
//! |                       |                                  |                      ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS
//! |                       |                                  |
//! |                       |                                  ETHERCAT_STATUS_DATA_ADDRESS
//! |                       ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
//! |
//! ETHERCAT_COMMAND_DATA_ADDRESS
//!
//!

//! Command/Status EtherCAT packet memory addresses. Necessary to Read/write data through Direct Memory Access (DMA)
#define PALM_0240_ETHERCAT_COMMAND_DATA_ADDRESS               0x1000
#define PALM_0240_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS    (PALM_0240_ETHERCAT_COMMAND_DATA_ADDRESS            + PALM_0240_ETHERCAT_COMMAND_DATA_SIZE)

#define PALM_0240_ETHERCAT_STATUS_DATA_ADDRESS                (PALM_0240_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS + ETHERCAT_CAN_BRIDGE_DATA_SIZE)
#define PALM_0240_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS     (PALM_0240_ETHERCAT_STATUS_DATA_ADDRESS             + PALM_0240_ETHERCAT_STATUS_DATA_SIZE)

//#define NUM_CONFIGS_REQUIRED 5

#endif
