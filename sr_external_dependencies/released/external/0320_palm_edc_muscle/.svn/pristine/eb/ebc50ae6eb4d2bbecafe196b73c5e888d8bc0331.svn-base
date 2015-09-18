//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        0320_palm_edc_ethercat_protocol.h
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

#ifndef PALM_EDC_0320_ETHERCAT_PROTOCOL_H_INCLUDED
#define PALM_EDC_0320_ETHERCAT_PROTOCOL_H_INCLUDED

#include "../common/tactile_edc_ethercat_protocol.h"
#include "../common/ethercat_can_bridge_protocol.h"
#include "../common/common_edc_ethercat_protocol.h"


#define NUM_MUSCLES                         40
#define NUM_PRESSURE_SENSORS_PER_MESSAGE     5
#define NUM_MUSCLE_DATA_PACKETS              8


// ========================================================
// 
//       F R O M    M U S C L E    D A T A    T Y P E
// 
// ========================================================


//! @author Hugo Elias
typedef enum
{
    MUSCLE_DATA_INVALID                  = 0x0,                  //!< For safety, this is not a valid request
    MUSCLE_DATA_PRESSURE                 = 0x1,                  //!< ADC reading pressure sensors

    MUSCLE_DATA_CAN_STATS                = 0x2,                  //!< Number of CAN messages received / transmitted by this muscle driver + error counters
    MUSCLE_DATA_SLOW_MISC                = 0x3                   //!< See FROM_MUSCLE_SLOW_DATA_TYPE
}FROM_MUSCLE_DATA_TYPE;




// ========================================================
// 
//                 S L O W    D A T A
// 
// ========================================================


typedef enum
{
    MUSCLE_SLOW_DATA_INVALID              = 0x0000,              //!< For safety, this is not a data type
    MUSCLE_SLOW_DATA_SVN_REVISION         = 0x0001,              //!< The revision of the code
    MUSCLE_SLOW_DATA_SVN_SERVER_REVISION  = 0x0002,              //!< The revision of the code on the SVN server at build time.
                                                                 //!  Should we have done an Update before building?
    MUSCLE_SLOW_DATA_SVN_MODIFIED         = 0x0003,              //!< Did the local code have any uncomitted modifications at build time?
    MUSCLE_SLOW_DATA_SERIAL_NUMBER_LOW    = 0x0004,              //!< 
    MUSCLE_SLOW_DATA_SERIAL_NUMBER_HIGH   = 0x0005,              //!< 
    MUSCLE_SLOW_DATA_ASSEMBLY_DATE_YYYY   = 0x0006,              //!< Year of assembly, E.G. 2012
    MUSCLE_SLOW_DATA_ASSEMBLY_DATE_MMDD   = 0x0007,              //!< Day/Month of assembly. E.G. 0x0A1F means October 31st


    MUSCLE_SLOW_DATA_LAST                 = 0x0007               //!< Important to know that this is the last one
}FROM_MUSCLE_SLOW_DATA_TYPE;



// ========================================================
// 
//                       F L A G S
// 
// ========================================================


                                                                // Non serious flags, Just for information. Control still works.
                                                                // -------------------------------------------------------------


                                                                // Serious flags cause the valves to be switched off
                                                                // ------------------------------------------------
#define MUSCLE_FLAG_BITS_NO_DEMAND_SEEN              0x0400     //!< Haven't received any demand messages for longer than NO_DEMAND_TIMEOUT_MS. Valves switched off.


#define PALM_0300_EDC_SERIOUS_ERROR_FLAGS            (  MOTOR_FLAG_BITS_NO_DEMAND_SEEN )

#define PALM_0300_EDC_NO_DEMAND_TIMEOUT_MS    20                                                              //!< If a muscle driver doesn't see any Torque or PWM demand values,
                                                                                                              //!  how long, in milliseconds, before it switches off the valves.

#ifndef NO_STRINGS													                            // The PIC compiler doesn't deal well with strings.

    //! These are the names of the bits in the MUSCLE_DATA_FLAGS.
    //! error_flag_names[n] is the name of bit 'n' in MUSCLE_DATA_FLAGS.
    static const char* palm_0300_edc_error_flag_names[16] = { "No demand seen for more than 20ms",            // 0x0400
                                                            };
#endif





// ========================================================
// 
//         T O    M U S C L E    D A T A    T Y P E
// 
// ========================================================

//! The host can send different types of data to the muscle drivers.
//! These can be either control demands, system messages.
//! These values are inserted into bits [3..0] of the message ID
//! in the muscle driver data message.
typedef enum
{
    MUSCLE_DEMAND_INVALID                = 0x0,                  //!< A zero is what happens if an EtherCAT packet doesn't get through, so it's considered a special case.
    MUSCLE_DEMAND_VALVES                 = 0x1,                  //!< Demanding valve pulses

    MUSCLE_SYSTEM_RESET                  = 0x3                   //!< Send with a demand value of 0x520x to reset muscle driver x
}TO_MUSCLE_DATA_TYPE;

#define MUSCLE_SYSTEM_RESET_KEY             0x5200               //!< | Muscle board ID.




#define MUSCLE_DEMAND_VALVES_RANGE_MIN     -0x4
#define MUSCLE_DEMAND_VALVES_RANGE_MAX      0x4

#define DIRECTION_TO_MUSCLE                 0x01
#define DIRECTION_FROM_MUSCLE               0x02


#ifndef NO_STRINGS													                    // The PIC compiler doesn't deal well with strings.

    //! These are the human-readable names of the different types of data.
    //! you can send to the muscle drivers.
    static const char* to_muscle_data_type_names[16] = { "INVALID",
                                                       "Demand: Valves",
                                                       };
#endif



//! Each muscle driver sends back five 12-bit pressure of status data on the CAN bus.
//! Generically, those two words look like this.

typedef union
{
    struct
    {
        unsigned int L:4;
        unsigned int M:4;
        unsigned int H:4;
        unsigned int padding:4;
    }   nibbles;

    int16u integer;

} TWELVE_BIT_INT_TO_NIBBLES;


typedef union
{
    struct 
    {
        unsigned int pressure0_L:4;
        unsigned int pressure0_M:4;
        unsigned int pressure0_H:4;

        unsigned int pressure1_L:4; 
        unsigned int pressure1_M:4;
        unsigned int pressure1_H:4;

        unsigned int pressure2_L:4;
        unsigned int pressure2_M:4;
        unsigned int pressure2_H:4;

        unsigned int pressure3_L:4;
        unsigned int pressure3_M:4;
        unsigned int pressure3_H:4;

        unsigned int pressure4_L:4;
        unsigned int pressure4_M:4;
        unsigned int pressure4_H:4;

        unsigned int padding:4;

    } packed;

    struct
    {
        int16u      SVN_revision;
        int16u      SVN_server;
        int16u      SVN_modified;
        int16u      nothing;
    }slow_0;

    struct
    {
        int32u      serial_number;
        int16u      assembly_date_YYYY;
        int8u       assembly_date_MM;
        int8u       assembly_date_DD;
    }slow_1;

    struct
    {
        int8u       can_err_tx;
        int8u       can_err_rx;
        int16u      can_msgs_tx;
        int16u      can_msgs_rx;
        int8u       nothing[2];
    }misc;

    //struct
    //{
        int8u raw[8];
    //} raw;
} MUSCLE_DATA_PACKET;



#define MESSAGE_ID_MUSCLE_DRIVER_ID_SHIFT_POS  5                       //!< Bit number of lowest bit of MESSAGE_ID_MUSCLE_ID_BITS



#define SENSORS_NUM_0320  ((int)36)                             //!< The number of sensors in the robot.
                                                                //!  This needs to be a #define because it's used to dimension an array.

#define JOINTS_NUM_0320   ((int)28)                             //!< The number of joints in the hand
                                                                //!  This needs to be a #define for symmetry with SENSORS_NUM



//#if (int)IGNORE > SENSORS_NUM
//    #error Not enough sensors[] in ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS
//#endif

//! These are the data sent from the Palm to the host.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< This tells us the contents of the data below.
                                                                    //!< This value should be identical to the EDC_command
                                                                    //!< value which arrived from the host in the previous
                                                                    //!< EtherCAT packet

	int16u					    sensors[SENSORS_NUM_0320+1];

    FROM_MUSCLE_DATA_TYPE       muscle_data_type;                   //!< Which data does pressure_sensors[] actually contain?
                                                                    //!< This value should agree with the previous value
                                                                    //!< in ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND

    int16u                       which_muscle_data_arrived;         //!< Bit s indicate which muscle data packet is new this frame
                                                                    //!  This variable needst to be 16 bits (even though we use
                                                                    //!  only 8 of the bits) to ensure that later words are 16-bit
                                                                    //!  aligned. (Mis-alignment causes memory exception).
    MUSCLE_DATA_PACKET          muscle_data_packet[NUM_MUSCLE_DATA_PACKETS];


    int32u                      tactile_data_type;
    int16u                      tactile_data_valid;                 //!< Bit 0: FF. Bit 4: TH.
    TACTILE_SENSOR_STATUS_v1    tactile[5];                         //

    int16u                      idle_time_us;                       //!< The idle time from when the palm has finished dealing with one EtherCAT
																    //!< packet, and the next packet arriving. Ideally, this number should be more than 50.

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS;



//! These are the data sent by the host.
typedef struct
{
    EDC_COMMAND                 EDC_command;                        //!< What type of data should the palm send back in the next packet?

    FROM_MUSCLE_DATA_TYPE       from_muscle_data_type;              //!< Which data does the host want from the muscles?

    TO_MUSCLE_DATA_TYPE         to_muscle_data_type;                //!< What type of data are we sending to the muscles?
    int8u                       muscle_data[NUM_MUSCLES/2];         //!< Data to send to muscles. [-4 .. 4] packed into the nibbles

    int32u                      tactile_data_type;                  //!< Request for specific tactile data
    //int32u                    tactile_data_type_1;                  //!< The host can request up to 2 different values per frame

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND;



#define PALM_0300_ETHERCAT_COMMAND_HEADER_SIZE  (  sizeof(EDC_COMMAND) + sizeof(FROM_MUSCLE_DATA_TYPE) + sizeof(int16s)  )

#define PALM_0300_ETHERCAT_STATUS_DATA_SIZE       sizeof(ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS)
#define PALM_0300_ETHERCAT_COMMAND_DATA_SIZE      sizeof(ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND)

                                                    //  Now we need to be *sure* that the Host and the Slave definitely
                                                    //  agree on the size of the EtherCAT packets, even if the host is a
                                                    //  64-bit machine or something. So we have these calculated sizes.
                                                    //  The host and slave can ASSERT that the sizeof() the packets
                                                    //  matches the agreed sizes.
#define ETHERCAT_STATUS_0300_AGREED_SIZE     236    //! This is the size of the Status  EtherCAT packet (Status + CAN packet)
#define ETHERCAT_COMMAND_0300_AGREED_SIZE    36     //! This is the size of the Command EtherCAT packet (Status + CAN packet)


//! | ETHERCAT_COMMAND_DATA | ETHERCAT_CAN_BRIDGE_DATA_COMMAND | ETHERCAT_STATUS_DATA | ETHERCAT_CAN_BRIDGE_DATA_STATUS |
//! |                       |                                  |                      |
//! |                       |                                  |                      ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS
//! |                       |                                  |
//! |                       |                                  ETHERCAT_STATUS_DATA_ADDRESS
//! |                       |
//! |                       ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
//! |
//! ETHERCAT_COMMAND_DATA_ADDRESS
//!
//!

#define PALM_0300_ETHERCAT_COMMAND_DATA_ADDRESS               0x1000
#define PALM_0300_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS    (PALM_0300_ETHERCAT_COMMAND_DATA_ADDRESS            + PALM_0300_ETHERCAT_COMMAND_DATA_SIZE)

#define PALM_0300_ETHERCAT_STATUS_DATA_ADDRESS                (PALM_0300_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS + ETHERCAT_CAN_BRIDGE_DATA_SIZE)
#define PALM_0300_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS     (PALM_0300_ETHERCAT_STATUS_DATA_ADDRESS             + PALM_0300_ETHERCAT_STATUS_DATA_SIZE)

//#define NUM_CONFIGS_REQUIRED 5


#endif
