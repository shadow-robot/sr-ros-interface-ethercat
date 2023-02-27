/*
* Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/
// FileName:        this_node.h
// Dependencies:
// Processor:       PIC32
// Compiler:        MPLAB® C32
//
//
//
//
//  Doxygen
//  -------
//
//! @file
//!
//! The term "Command" means data going from the ROS PC to the Node on the robot
//! Previously known as "Incoming"
//!
//! The term "Status"  means data going from Node on the robot the to the ROS PC
//! Previously known as "Outgoing"
//!
//!
//! @addtogroup
//

#ifndef PALM_EDC_0250_ETHERCAT_PROTOCOL_H_INCLUDED
#define PALM_EDC_0250_ETHERCAT_PROTOCOL_H_INCLUDED

#include "../0220_palm_edc/0220_palm_edc_ethercat_protocol.h"

// #define  IMU_COMMAND_NONE        0
// #define  IMU_COMMAND_SET_SCALE   1

// typedef struct
// {
//   int16u command;
//   int8u  argument[2];
// } IMU_COMMAND_TYPE;

//! These are the data sent from the Palm to the host.
typedef struct
{
  EDC_COMMAND                 EDC_command;                        //!< This tells us the contents of the data below.
                                                                    //!< This value should be identical to the EDC_command
                                                                    //!< value which arrived from the host in the previous
                                                                    //!< EtherCAT packet

	int16u					            sensors[SENSORS_NUM_0220+1];          //!<          74 bytes

  FROM_MOTOR_DATA_TYPE        motor_data_type;                    //!< Which data does motor[] contain?
                                                                  //!< This value should agree with the previous value
                                                                  //!< in ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND
  int16s                      which_motors;                       //!< 0: Even motor numbers.  1: Odd motor numbers
                                                                  //!< This value should agree with the previous value
                                                                  //!< in ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND

  int32u                      which_motor_data_arrived;           //!< Bit N set when motor CAN message arrives. Ideally, bits 0..19 get set
  int32u                      which_motor_data_had_errors;        //!< Bit N set when motor sends bad CAN message Ideally, no bits get set.

  MOTOR_DATA_PACKET           motor_data_packet[10];              //!< Data for 10 motors only. (Even ones or Odd ones)
                                                                  //  TOTAL MOTOR DATA = 40 bytes

  int32u                      tactile_data_type;                  //!<           4 bytes
  int16u                      tactile_data_valid;                 //!<           2 bytes          (Bit 0: FF. Bit 4: TH.)
  TACTILE_SENSOR_STATUS_v3    tactile[5];                         // 104*5 = 520 bytes
                                                                  //!< TOTAL = 396 bytes

  int16u                      idle_time_us;                       //!< The idle time from when the palm has finished dealing with one EtherCAT
                                                                    //!< packet, and the next packet arriving. Ideally, this number should be more than 50.

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS;


//! These are the data sent by the host.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< What type of data should the palm send back in the next packet?

    FROM_MOTOR_DATA_TYPE        from_motor_data_type;               //!< Which data does the host want from the motors?
    int16s                      which_motors;                       //!< Which motors does the host want to read?
                                                                    //!< 0: Even motor numbers.  1: Odd motor numbers

    TO_MOTOR_DATA_TYPE          to_motor_data_type;
    int16s                      motor_data[NUM_MOTORS];             //!< Data to send to motors. Typically torque/PWM demands, or configs.

    int32u                      tactile_data_type;                  //!< Request for specific tactile data
    IMU_COMMAND_TYPE            imu_command;                        // Command to configure the IMU

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND;

/*
#define ETHERCAT_COMMAND_HEADER_SIZE  (  sizeof(EDC_COMMAND)           \       //!< What's the minimum amount of
                                       + sizeof(FROM_MOTOR_DATA_TYPE)   \
                                       + sizeof(int16s)  )
*/

#define PALM_0250_ETHERCAT_COMMAND_HEADER_SIZE  (  sizeof(EDC_COMMAND) + sizeof(FROM_MOTOR_DATA_TYPE) + sizeof(int16s)  )

#define PALM_0250_ETHERCAT_STATUS_DATA_SIZE       sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS)
#define PALM_0250_ETHERCAT_COMMAND_DATA_SIZE      sizeof(ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND)

                                                //  Now we need to be *sure* that the Host and the Slave definitely
                                                //  agree on the size of the EtherCAT packets, even if the host is a
                                                //  64-bit machine or something. So we have these calculated sizes.
                                                //  The host and slave can ASSERT that the sizeof() the packets
                                                //  matches the agreed sizes.
#define ETHERCAT_STATUS_0250_AGREED_SIZE     312//Before was  232! This is the size of the Status  EtherCAT packet (Status + CAN packet) 
#define ETHERCAT_COMMAND_0250_AGREED_SIZE    74      //! This is the size of the Command EtherCAT packet (Status + CAN packet)



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

#define PALM_0250_ETHERCAT_COMMAND_DATA_ADDRESS               0x1000
#define PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS    (PALM_0250_ETHERCAT_COMMAND_DATA_ADDRESS            + PALM_0250_ETHERCAT_COMMAND_DATA_SIZE)

#define PALM_0250_ETHERCAT_STATUS_DATA_ADDRESS                (PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS + ETHERCAT_CAN_BRIDGE_DATA_SIZE)
#define PALM_0250_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS     (PALM_0250_ETHERCAT_STATUS_DATA_ADDRESS             + PALM_0250_ETHERCAT_STATUS_DATA_SIZE)

//#define NUM_CONFIGS_REQUIRED 5


#endif
