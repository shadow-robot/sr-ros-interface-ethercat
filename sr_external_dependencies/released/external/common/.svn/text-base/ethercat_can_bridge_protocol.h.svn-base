//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        ethercat_can_bridge_protocol.h
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

#ifndef ETHERCAT_CAN_BRIDGE_PROTOCOL_H_INCLUDED
#define ETHERCAT_CAN_BRIDGE_PROTOCOL_H_INCLUDED


//! This packet allows the palm to transmit and receive CAN messages
//! on either CAN bus. One CAN message per EtherCAT packet only.
//! The CAN messages can be used for bootloading new code onto the motors,
//! or to configure the motor boards.
typedef struct
{
    int8u   can_bus;
    int8u   message_length;
    int16u  message_id;
    int8u   message_data[8];
} __attribute__((packed)) ETHERCAT_CAN_BRIDGE_DATA;

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE   sizeof(ETHERCAT_CAN_BRIDGE_DATA)


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



#endif
