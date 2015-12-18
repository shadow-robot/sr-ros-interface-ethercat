//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        common_edc_ethercat_protocol.h
// Dependencies:
// Processor:       PIC32
// Compiler:        MPLAB® C32
//
//  +------------------------------------------------------------------------+
//  | This file is part of The Shadow Robot PIC32 firmware code base.        |
//  |                                                                        |
//  | It is free software: you can redistribute it and/or modify             |
//  | it under the terms of the GNU General Public License as published by   |
//  | the Free Software Foundation, either version 3 of the License, or      |
//  | (at your option) any later version.                                    |
//  |                                                                        |
//  | It is distributed in the hope that it will be useful,                  |
//  | but WITHOUT ANY WARRANTY; without even the implied warranty of         |
//  | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          |
//  | GNU General Public License for more details.                           |
//  |                                                                        |
//  | You should have received a copy of the GNU General Public License      |
//  | along with this code repository. The text of the license can be found  |
//  | in Pic32/License/gpl.txt. If not, see <http://www.gnu.org/licenses/>.  |
//  +------------------------------------------------------------------------+
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

#ifndef COMMON_EDC_ETHERCAT_PROTOCOL_H_INCLUDED
#define COMMON_EDC_ETHERCAT_PROTOCOL_H_INCLUDED

//! The host can request different types of data from the palm.
typedef enum
{
    EDC_COMMAND_INVALID = 0,                                    //!< Reading an empty mailbox on the ET1200 results in a zero.
    EDC_COMMAND_SENSOR_DATA,                                    //!< Normal operating value. Palm transmits ADC readings.
    EDC_COMMAND_SENSOR_CHANNEL_NUMBERS,                         //!< Instead of sending ADC readings, send the channel number, so the host can confirm the firmware is correct.
    EDC_COMMAND_SENSOR_ADC_CHANNEL_CS,                          //!< Instead of sending ADC readings, send the chip select channel, so the host can confirm the firmware is correct.
    EDC_COMMAND_CAN_DIRECT_MODE                                 //!< Might be used in the future for running automated tests inside the firmware.

    //EDC_COMMAND_TEST_RESULTS,                                   //!< Might be used in the future for running automated tests inside the firmware.
}EDC_COMMAND;



#ifndef NO_STRINGS													                        // The PIC compiler doesn't deal well with strings.

    static const char* slow_data_types[17] = {  "Invalid",                                  // 0x0000
                                                "SVN revision",                             // 0x0001
                                                "SVN revision on server at build time",     // 0x0002

                                                "Modified from SVN revision",               // 0x0003
                                                "Serial number low",                        // 0x0004
                                                "Serial number high",                       // 0x0005
                                                "Motor gear ratio",                         // 0x0006
                                                "Assembly date year",                       // 0x0007
                                                "Assembly date month, day",                 // 0x0008

                                                "Controller F",                             // 0x0009
                                                "Controller P",                             // 0x000A
                                                "Controller I",                             // 0x000B
                                                "Controller Imax",                          // 0x000D
                                                "Controller D",                             // 0x000E
                                                "Controller deadband and sign",             // 0x000F

                                                "Controller loop frequency Hz"              // 0x0010
                                               };

#endif


//! This represents the top two bits [10..9] of the CAN message ID.
//! These bits tell us the type of the message.
typedef enum
{
    DIRECTION_DATA_REQUEST              = 0x0,                  //!< Requesting that motors send back status data (AKA Start of Frame)
    DIRECTION_TO_MOTOR                  = 0x1,                  //!< Message contains command data being sent to the motors
    DIRECTION_FROM_MOTOR                = 0x2,                  //!< Message contains status data from a motor
    DIRECTION_BOOTLOADER                = 0x3                   //!< Message has something to do with boot-loading.
}MESSAGE_DIRECTION;


#define MESSAGE_ID_DIRECTION_BITS       0b11000000000           //!< Bit mask specifying which bits of the CAN message ID are used for the MESSAGE_DIRECTION
#define MESSAGE_ID_MOTOR_ID_BITS        0b00111100000           //!< Bit mask specifying which bits of the CAN message ID are used for the motor ID [0..9]
#define MESSAGE_ID_ACK_BIT              0b00000010000           //!< Bit mask specifying which bits of the CAN message ID are used for the ACK bit (only for bootloading)
#define MESSAGE_ID_TYPE_BITS            0b00000001111           //!< Bit mask specifying which bits of the CAN message ID are used for the TO_MOTOR_DATA_TYPE or FROM_MOTOR_DATA_TYPE
                                                                //                                               or for the TO_MUSCLE_DATA_TYPE or FROM_MUSCLE_DATA_TYPE

#define MESSAGE_ID_DIRECTION_SHIFT_POS  9                       //!< Bit number of lowest bit of MESSAGE_ID_DIRECTION_BITS


//Check this against the SENSORS_NUM_0320 and SENSORS_NUM_0220
#define SENSORS_NUM_0X20  ((int)36)                             //!< The number of sensors in the robot.
                                                                //!  This needs to be a #define because it's used to dimension an array.

//Check this against the JOINTS_NUM_0320 and JOINTS_NUM_0220
#define JOINTS_NUM_0X20   ((int)28)                             //!< The number of joints in the hand
                                                                //!  This needs to be a #define for symmetry with SENSORS_NUM



#ifndef NO_STRINGS                                              //   The PIC compiler doesn't deal well with strings.

    static const char* joint_names[JOINTS_NUM_0X20] = {  "FFJ0", "FFJ1", "FFJ2", "FFJ3", "FFJ4",
                                                         "MFJ0", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                                         "RFJ0", "RFJ1", "RFJ2", "RFJ3", "RFJ4",
                                                         "LFJ0", "LFJ1", "LFJ2", "LFJ3", "LFJ4","LFJ5",
                                                         "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                                                         "WRJ1", "WRJ2"
                                                      };


    //! This array defines the names of the joints. The names and order should match the enum SENSOR_NAMES_ENUM.

    static const char* sensor_names[SENSORS_NUM_0X20] = {"FFJ1",  "FFJ2",  "FFJ3", "FFJ4",	                     //  [00..03] ADC readings from First finger
                                                         "MFJ1",  "MFJ2",  "MFJ3", "MFJ4",                       //  [04..07] ADC readings from Middle finger
                                                         "RFJ1",  "RFJ2",  "RFJ3", "RFJ4",                       //  [08..11] ADC readings from Ring finger
                                                         "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5",               //  [12..16] ADC readings from Little finger
                                                         "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",     //  [17..22] ADC readings from Thumb
                                                         "WRJ1A", "WRJ1B", "WRJ2",                               //  [23..25] ADC readings from Wrist
                                                         "ACCX",  "ACCY",  "ACCZ",                               //  [26..28] ADC readings from Accelerometer
                                                         "GYRX",  "GYRY",  "GYRZ",                               //  [29..31] ADC readings from Gyroscope
                                                         "AN0",   "AN1",   "AN2",  "AN3"                         //  [32..35] ADC readings from auxillary ADC port.
                                                       };
#endif


//! This enum defines which ADC reading goes into which sensors[].
typedef enum
{
	FFJ1=0, FFJ2,  FFJ3, FFJ4,                      // [ 0...3]
	MFJ1,   MFJ2,  MFJ3, MFJ4,                      // [ 4...7]
	RFJ1,   RFJ2,  RFJ3, RFJ4,                      // [ 8..11]
	LFJ1,   LFJ2,  LFJ3, LFJ4, LFJ5,                // [12..16]
    THJ1,   THJ2,  THJ3, THJ4, THJ5A, THJ5B,        // [17..22]
    WRJ1A,  WRJ1B, WRJ2,                            // [23..25]

	ACCX, ACCY, ACCZ,                               // [26..28]
	GYRX, GYRY, GYRZ,                               // [29..32]

	ANA0, ANA1, ANA2, ANA3,                     // [31..35]
    IGNORE                                          // [36]
}SENSOR_NAME_ENUM;




typedef enum
{
      PALM_SVN_VERSION              =  0,
    SERVER_SVN_VERSION              =  1
}HARD_CONFIGURATION_INFORMATION;



#define INSERT_CRC_CALCULATION_HERE 	crc_i = (int8u) (crc_result&0xff);          \
                                    	crc_i ^= crc_byte;                          \
                                    	crc_result >>= 8;                           \
                                    	if(crc_i & 0x01)	crc_result ^= 0x3096;   \
                                    	if(crc_i & 0x02)	crc_result ^= 0x612c;   \
                                    	if(crc_i & 0x04)	crc_result ^= 0xc419;   \
                                    	if(crc_i & 0x08)	crc_result ^= 0x8832;   \
                                    	if(crc_i & 0x10)	crc_result ^= 0x1064;   \
                                    	if(crc_i & 0x20)	crc_result ^= 0x20c8;   \
                                    	if(crc_i & 0x40)	crc_result ^= 0x4190;   \
                                    	if(crc_i & 0x80)	crc_result ^= 0x8320;


#endif
