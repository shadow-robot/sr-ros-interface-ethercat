/* 
* Copyright 2023-2024 Shadow Robot Company Ltd.
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
  * @file 0250_palm_edc_IMU_ethercat_protocol.h
  *
  * @brief EtherCat protocol for devices with Product ID 10
  *
  *  The Ethercat "Command" packet refers to data sent from the ROS host to the Node on the robot
  *  The Ethercat "Status" packet refers to data sent from Node on the robot the to the ROS host.
  */

#ifndef PALM_EDC_0250_ETHERCAT_PROTOCOL_H_INCLUDED
#define PALM_EDC_0250_ETHERCAT_PROTOCOL_H_INCLUDED

#include "../0220_palm_edc/0220_palm_edc_ethercat_protocol.h"

//! Data structure sent from the Palm to the host (Status).
//! Data is written to the EtherCAT bus sequencially, in the same order defined below.
typedef struct
{
  EDC_COMMAND                 EDC_command;                  //!< Identifies the contents of the data below and should be identical to the EDC_command
                                                            //!< value which arrived from the host in the previous EtherCAT packet
                                                            // 4 bytes

  int16u					            sensors[SENSORS_NUM_0220+1];  //!< Joints sensors data
                                                            // 74 bytes

  int32u                      tactile_data_type;            //!< Identifies the tactile data type
                                                            // 4 bytes
  int16u                      tactile_data_valid;           //!< Identifies if tactile data is valid for each finger (Bit 0: FF. Bit 4: TH)
                                                            // 2 bytes
  TACTILE_SENSOR_STATUS_v3    tactile[5];                   //!< Tacile sensors data
                                                            // 78 * 5 = 390 bytes
                                                            // TOTAL Tactile data = 396 bytes

  FROM_MOTOR_DATA_TYPE        motor_data_type;              //!< Identifies the contents present in motor_data_packet[] below
                                                            //!< This value should be the same as the one sent in the previous command
                                                            //!< ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND
                                                            // 4 bytes
                                                                  
  int16s                      which_motors;                 //!< Identifies the group of motors sending data in motor_data_packet[] below.
                                                            //!< 0: Even motor numbers.  1: Odd motor numbers
                                                            //!< This value should be the same as the one sent in the previous command
                                                            //!< ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND                                                              
                                                            // 2 bytes

  int32u                      which_motor_data_arrived;     //!< Bit N set when motor CAN message arrives.
                                                            //!< Ideally, bits 0..19 get set
                                                            // 4 bytes     

  int32u                      which_motor_data_had_errors;  //!< Bit N set when motor sends bad CAN message.
                                                            //!< Ideally, no bits get set.
                                                            // 4 bytes     

  MOTOR_DATA_PACKET           motor_data_packet[10];        //!< Data from the motors. 
                                                            //!< Only 10 motors send data at a time (even or odd ones).
                                                            // 4 * 10 = 40 bytes
                                                            // Total Motor data = 54 bytes

  int16u                      idle_time_us;                 //!< Time distance between the previous EtherCAT Status being sent, and next Command packet arriving.
                                                            //!< Ideally, this number should be more than 50.
                                                            // 2 bytes

                                                            // Total: 530 bytes

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS;


//! Data structure sent from the host to the Palm (Command).
//! Data is written to the EtherCAT bus sequencially, in the same order defined below.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< Identifies the data being requested to the Palm
                                                                    // 4 bytes

    FROM_MOTOR_DATA_TYPE        from_motor_data_type;               //!< Identifies the type of data we want to read from the motors
                                                                    // 4  bytes
    
    int16s                      which_motors;                       //!< Identifies the group of motors (odd or even) we want to read data from.
                                                                    //!< 0: Even motor numbers. 1: Odd motor numbers
                                                                    // 2  bytes

    TO_MOTOR_DATA_TYPE          to_motor_data_type;                 //!< Identifies the command/data type being sent to the motors
                                                                    // 4  bytes
    
    int16s                      motor_data[NUM_MOTORS];             //!< Data to send to the motors. Typically torque/PWM demands, or configs.
                                                                    // 2 * 20 = 40 bytes

    int32u                      tactile_data_type;                  //!< Identifies the type of data we want to read from the tactile sensors
                                                                    // 4  bytes
    
    IMU_COMMAND_TYPE            imu_command;                        //!< Identifies the command being sent to the IMU.
                                                                    //!< For IMU configuration purposes.
                                                                    // 4 bytes

                                                                    // Total: 62 bytes

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND;

//! EtherCAT protocol packet/header sizes
#define PALM_0250_ETHERCAT_COMMAND_HEADER_SIZE  (sizeof(EDC_COMMAND) + sizeof(FROM_MOTOR_DATA_TYPE) + sizeof(int16s))

#define PALM_0250_ETHERCAT_STATUS_DATA_SIZE       sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS)
#define PALM_0250_ETHERCAT_COMMAND_DATA_SIZE      sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND)

//! Ethercat Command and Status packets "agreed" sizes.
//! They are use by the host and clients to assert if the incoming packets are the correct size.
#define ETHERCAT_STATUS_0250_AGREED_SIZE     542  //!< This is the size of the Status  EtherCAT packet (Status + CAN packet) 
#define ETHERCAT_COMMAND_0250_AGREED_SIZE    74   //!< This is the size of the Command EtherCAT packet (Command + CAN packet)


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
#define PALM_0250_ETHERCAT_COMMAND_DATA_ADDRESS               0x1000
#define PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS    (PALM_0250_ETHERCAT_COMMAND_DATA_ADDRESS            + PALM_0250_ETHERCAT_COMMAND_DATA_SIZE)

#define PALM_0250_ETHERCAT_STATUS_DATA_ADDRESS                (PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS + ETHERCAT_CAN_BRIDGE_DATA_SIZE)
#define PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS     (PALM_0250_ETHERCAT_STATUS_DATA_ADDRESS             + PALM_0250_ETHERCAT_STATUS_DATA_SIZE)

//#define NUM_CONFIGS_REQUIRED 5

#endif
