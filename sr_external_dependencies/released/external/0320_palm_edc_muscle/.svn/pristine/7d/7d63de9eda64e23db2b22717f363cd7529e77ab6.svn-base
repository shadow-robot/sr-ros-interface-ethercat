//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        this_node.c
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
//
//  Doxygen
//  -------
//
//! @file
//
//! @group Which group this documentation should go in.
//

#include "this_node.h"
#include "tests/assert_shadow.h"
#include "internal_reporting/internal_reporting.h"
#include "hardware/et1200/et1200_interface.h"
#include <peripheral/spi.h>

int64u      node_id                  = 0;
int32u      global_AL_Event_Register = 0;

TACTILE_SENSOR_PROTOCOL_TYPE    tactile_sensor_protocol = TACTILE_SENSOR_PROTOCOL_TYPE_INVALID;

ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND   etherCAT_command_data;              //!< Data structure containing command data, arrived from host PC.
ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS    etherCAT_status_data;               //!< Data structure containing sensor data, going to PC.

ETHERCAT_CAN_BRIDGE_DATA                        can_bridge_data_from_ROS;
ETHERCAT_CAN_BRIDGE_DATA                        can_bridge_data_to_ROS;


#define CORE_TIMER_TO_MICROSECONDS(x) ((x) / (SYSTEM_FREQ_HZ/2000000))              //!< Convert from the number of core ticks returned by ReadCoreTimer to a real time in microseconds.

#define ETHERCAT_COMMAND_DATA_ADDRESS PALM_0300_ETHERCAT_COMMAND_DATA_ADDRESS
#define ETHERCAT_COMMAND_DATA_SIZE PALM_0300_ETHERCAT_COMMAND_DATA_SIZE
#define ETHERCAT_STATUS_DATA_ADDRESS PALM_0300_ETHERCAT_STATUS_DATA_ADDRESS
#define ETHERCAT_STATUS_DATA_SIZE PALM_0300_ETHERCAT_STATUS_DATA_SIZE
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS PALM_0300_ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS PALM_0300_ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS


#ifdef PALM_PCB_00
    int8u palm_EDC_0300_sensor_mapping[64] = {IGNORE, IGNORE, IGNORE, IGNORE,  FFJ4,   THJ3,   THJ1,    IGNORE,           // Channel 0
                                              IGNORE, IGNORE, IGNORE, IGNORE,  MFJ4,   THJ4,   THJ2,    IGNORE,           // Channel 1
                                              FFJ2  , MFJ2  , RFJ2  , LFJ2  ,  RFJ4,   AN0,    IGNORE,  IGNORE,           // Channel 2
                                              FFJ1  , MFJ1  , RFJ1  , LFJ1  ,  THJ5B,  THJ5A,  IGNORE,  IGNORE,           // Channel 3
                                              IGNORE, IGNORE, IGNORE, IGNORE,  WRJ1B,  WRJ1A,  IGNORE,  IGNORE,           // Channel 4
                                              IGNORE, IGNORE, IGNORE, IGNORE,  AN1,    LFJ4,   IGNORE,  IGNORE,           // Channel 5
                                              FFJ3  , MFJ3  , RFJ3  , LFJ3  ,  AN2,    LFJ5,   IGNORE,  IGNORE,           // Channel 6
                                              IGNORE, IGNORE, IGNORE, IGNORE,  AN3,    WRJ2,   IGNORE,  IGNORE            // Channel 7
                                             };
#endif



#ifdef PALM_PCB_01    
    int8u palm_EDC_0300_sensor_mapping[64] = {IGNORE, IGNORE, IGNORE, IGNORE,  FFJ4,   THJ4,   IGNORE,  IGNORE,           // Channel 0
                                              IGNORE, IGNORE, IGNORE, IGNORE,  MFJ4,   THJ3,   IGNORE,  IGNORE,           // Channel 1
                                              RFJ2  , MFJ2  , FFJ2  , LFJ2  ,  RFJ4,   AN0,    THJ1,    IGNORE,           // Channel 2
                                              RFJ1  , MFJ1  , FFJ1  , LFJ1  ,  THJ5B,  THJ5A,  IGNORE,  IGNORE,           // Channel 3
                                              IGNORE, IGNORE, IGNORE, IGNORE,  AN3,    WRJ2,   THJ2,    IGNORE,           // Channel 4
                                              IGNORE, IGNORE, IGNORE, IGNORE,  AN2,    LFJ4,   IGNORE,  IGNORE,           // Channel 5
                                              RFJ3  , MFJ3  , FFJ3  , LFJ3  ,  AN1,    LFJ5,   IGNORE,  IGNORE,           // Channel 6
                                              IGNORE, IGNORE, IGNORE, IGNORE,  WRJ1A,  WRJ1B,  IGNORE,  IGNORE            // Channel 7
                                             };
#endif



int32u frame_start_time = 0;                //!< Is set to the value of the MIPS core timer when an EtherCAT packet arrives
int32u idle_time_start  = 0;                //!< Is set to the value of the MIPS core timer when the status data are written to the ET1200




LED_Class *LED_CAN1_TX;                     //!< The CAN 1 transmit LED
LED_Class *LED_CAN1_RX;                     //!< The CAN 1 receive LED
LED_Class *LED_CAN1_ERR;                    //!< The CAN 1 error LED

LED_Class *LED_CAN2_TX;                     //!< The CAN 2 transmit LED
LED_Class *LED_CAN2_ERR;                    //!< The CAN 2 error LED

LED_Class *ET1200_AL_err_LED;               //!< EtherCAT Application Layer Error LED. See EtherCAT Indicator Specification V0.91.pdf section 4.5 ERR Indicator.


#ifdef LED_CFG_PIN
  LED_Class  *LED_CFG;                      //!< This LED is optional, and exists if LED_CFG_PIN is defined.
                                            //!  It's lit when a board implementing the old CAN Shadow Protocol switches into config mode.
                                            //!  If it's not defined, then LED_CAN2_RX probably is.
#endif


#ifdef LED_CAN2_RX_PIN
    LED_Class *LED_CAN2_RX;                 //!< This LED is optional, and exists if LED_CAN2_RX_PIN is defined.
                                            //!  It's the CAN 2 receive LED.
                                            //!  If it's not defined, then LED_CFG probably is.
#endif


int num_muscle_CAN_messages_received_this_frame = 0;     //!< Count of the number of CAN messages received since the Start Of Frame message was sent.



void run_tests(void)
{
    typedef_tests();
    simple_CAN_tests();
}



//! This function should be called about once every millisecond.
//! It can either be called once for every EtherCAT packet, or
//! just based on a timer.
//! If this function isn't called, then the Watch Dog timer will time out.
//!
//! @author Hugo Elias
void approx_1ms_handler(void)
{
    Handle_LEDs();

    #if AUTO_TRIGGER == 1
        Read_All_Sensors();
    #endif

    ClearWDT();
}



//! Initialise the hardware of this node.
//! * I/O pins
//! * SPI
//! * I2C
//! * LEDs
//! * ET1200 interface
//! * ET1200
//! * CAN buses
//! * parallel SPI
//! * tactile sensors
//! 
//! @author Hugo Elias
void initialise_this_node(void)
{
    run_tests();                                                                            // Very important


                                                                                            // Initialise all of the I/O pins and peripherals.
    Output_Pin_Class *ET1200_chip_select    = Get_Output_Pin(ET1200_CHIP_SELECT_PIN);
    Output_Pin_Class *ET1200_reset          = Get_Output_Pin(ET1200_RESET_PIN);
    Input_Pin_Class  *ET1200_eeprom         = Get_Input_Pin (ET1200_EEPROM_PIN);

    SPI_simple_master_class   *SPI_port     = Init_SPI_Simple_Master(SPI_PORT);
    I2C_simple_master_class   *I2C_port     = Init_I2C_Simple_Master(I2C_PORT);

    Output_Pin_Class *SPI_cs                = Get_Output_Pin(SPI_CS_PIN);
    Output_Pin_Class *SPI_clock             = Get_Output_Pin(SPI_CLOCK_PIN);
    Output_Pin_Class *SPI_mosi              = Get_Output_Pin(SPI_MOSI_PIN);
    Input_Pin_Class  *ET1200_somi           = Get_Input_Pin (ET1200_SOMI_PIN);


    Output_Pin_Class *LED_pin_CAN1_TX       = Get_Output_Pin(LED_CAN1_TX_PIN);
    Output_Pin_Class *LED_pin_CAN1_RX       = Get_Output_Pin(LED_CAN1_RX_PIN);
    Output_Pin_Class *LED_pin_CAN1_ERR      = Get_Output_Pin(LED_CAN1_ERR_PIN);

    Output_Pin_Class *LED_pin_CAN2_TX       = Get_Output_Pin(LED_CAN2_TX_PIN);
    Output_Pin_Class *LED_pin_CAN2_ERR      = Get_Output_Pin(LED_CAN2_ERR_PIN);

    Output_Pin_Class *LED_pin_AL_ERR        = Get_Output_Pin(LED_AL_ERR_PIN);

    #ifdef ACCEL_CS_PIN
        Output_Pin_Class *accel_CS          = Get_Output_Pin(ACCEL_CS_PIN);
        Pin_Set(accel_CS);
    #endif

    LED_CAN1_TX                             = Get_LED(LED_pin_CAN1_TX);
    LED_CAN1_RX                             = Get_LED(LED_pin_CAN1_RX);
    LED_CAN1_ERR                            = Get_LED(LED_pin_CAN1_ERR);

    LED_CAN2_TX                             = Get_LED(LED_pin_CAN2_TX);
    LED_CAN2_ERR                            = Get_LED(LED_pin_CAN2_ERR);

    ET1200_AL_err_LED                       = Get_LED(LED_pin_AL_ERR);

    #ifdef LED_CFG_PIN
    	Output_Pin_Class *LED_pin_CFG       = Get_Output_Pin(LED_CFG_PIN);
    	LED_CFG                             = Get_LED(LED_pin_CFG);
    #endif

    #ifdef LED_CAN2_RX_PIN
    	Output_Pin_Class *LED_pin_CAN2_RX   = Get_Output_Pin(LED_CAN2_RX_PIN);
    	LED_CAN2_RX                         = Get_LED(LED_pin_CAN2_RX);
    #endif


    AD1PCFG = 0xFFFF;                                                                       // Set all pins to Digital, not analog.
    U1CON   = 0x0000;                                                                       // Disable USB


    ET1200_Interface_Initialise(SPI_port, I2C_port, ET1200_reset, ET1200_AL_err_LED, ET1200_eeprom, ET1200_somi, ET1200_chip_select);

    #if AUTO_TRIGGER == 0
        ET1200_Initialise();
        write_ET1200_register_32u(0x204, 0x100);                                            //set AL event mask to get IRQ only on SyncMan0 activity
    #endif


    init_dual_CAN();




    {                                                                                       // Initialise the MOSI bits for each ADC channel
        Input_Pins_Class *SPI_SOMI_pins      = Get_Input_Pins('E', 0, 8);
    
        Initialise_SPI_Parallel(SPI_cs, SPI_clock, SPI_mosi, SPI_SOMI_pins);
        SPIP_Setup_MOSI_Bits(0, 19, 0xC0000000);        // 1100 0000
        SPIP_Setup_MOSI_Bits(1, 19, 0xC8000000);        // 1100 1000
        SPIP_Setup_MOSI_Bits(2, 19, 0xD0000000);        // 1101 0000
        SPIP_Setup_MOSI_Bits(3, 19, 0xD8000000);        // 1101 1000
        SPIP_Setup_MOSI_Bits(4, 19, 0xE0000000);        // 1110 0000
        SPIP_Setup_MOSI_Bits(5, 19, 0xE8000000);        // 1110 1000
        SPIP_Setup_MOSI_Bits(6, 19, 0xF0000000);        // 1111 0000
        SPIP_Setup_MOSI_Bits(7, 19, 0xF8000000);        // 1111 1000
    }


    tactile_sensor_protocol = biotac_autodetect(tactile_sensor_protocol);                   // Auto-detect the tactile sensor type.
    tactile_sensor_protocol =    pst_autodetect(tactile_sensor_protocol);                   // Result is stored in tactile_sensor_protocol.

    switch (tactile_sensor_protocol)                                                        // Now initialise the tactile sensors according to the detected type.
    {
        case TACTILE_SENSOR_PROTOCOL_TYPE_PST3:             init_pst();         break;      // PST3     (new type with normal CS)
        case TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3:       init_biotac();      break;      // BioTac   (Syntouch)

        default:                                                                break;
    }


}



void Read_Commands_From_ET1200(void)
{
    assert_dynamic(ETHERCAT_COMMAND_DATA_SIZE > 0);
    //read_ET1200_register_N(EC_PALM_EDC_COMMAND_PHY_BASE, ETHERCAT_COMMAND_DATA_SIZE, (int8u*)(&etherCAT_command_data));
    /*read_ET1200_register_N( ETHERCAT_COMMAND_DATA_ADDRESS,
                            ETHERCAT_COMMAND_DATA_SIZE,
                            (int8u*)(&etherCAT_command_data)  );*/
    read_DMA_ET1200_register_N ( ETHERCAT_COMMAND_DATA_ADDRESS,
                             ETHERCAT_COMMAND_DATA_SIZE,
                            (int8u*)(&etherCAT_command_data)  );
}



//! Write the whole etherCAT_status_data[] array to the ET1200.
//! Once these data are written, we are ready for the next EtherCAT
//! packet to arrive, and the idle timer starts.
//! 
//! @author Hugo Elias
inline void write_status_data_To_ET1200(void)
{
    //write_ET1200_register_N(ETHERCAT_STATUS_DATA_ADDRESS, ETHERCAT_STATUS_DATA_SIZE, (int8u*)(&etherCAT_status_data));
    write_DMA_ET1200_register_N(ETHERCAT_STATUS_DATA_ADDRESS, ETHERCAT_STATUS_DATA_SIZE, (int8u*)(&etherCAT_status_data));
}



//! Here we check the content of AL Event Register (0x220)
//! We check the bit 8, which is "SyncManager0 interrupt pending"
//! SyncManager0 is the Command SyncManager. See II-36 of the ET1200 datasheet.
//! This function reads the Global AL Event register (0x220).
//! 
//! @author Yann Sionneau
int8u There_is_Command_From_ET1200(void)
{
    global_AL_Event_Register = read_ET1200_register_32u(0x220);
    return ( ( global_AL_Event_Register & ECREG32_AL_EVENT_BIT_SYNCMANAGER_0_INTERRUPT ) != 0) ;
}



//! Here we check the content of AL Event Register (0x220)
//! We check the bit 9, which is "SyncManager1 interrupt pending"
//! SyncManager0 is the Command SyncManager. See II-36 of the ET1200 datasheet.
//! This function DOES NOT read the Global AL Event register (0x220). It assumes the
//! register has already been read
//! 
//! @author Yann Sionneau
int8u ROS_Wants_me_to_send_CAN(void)
{
    return ( ( global_AL_Event_Register & ECREG32_AL_EVENT_BIT_SYNCMANAGER_1_INTERRUPT ) != 0) ;
}



//! Are we still waiting for CAN messages to arrive?
//!
//! @author Hugo Elias
int8u not_all_muscle_data_received(void)
{
    return num_muscle_CAN_messages_received_this_frame < NUM_MUSCLE_DATA_PACKETS;
}



//! Return the time in microseconds since the the beginning of the current 1ms frame.
//! The timer is started when the PIC sees that new Command data are available.
//!
//! @author Hugo Elias
int32u get_frame_time_us(void)
{
    return (ReadCoreTimer() - frame_start_time) / (SYSTEM_FREQ_HZ/2000000);
}



//! The muscle drivers will each send back two data messages.
//! Wait until we have 2 messages from each bus, then we
//! can write the data back to the ET1200
//!
//! But, at some point, we have to time out, because either some
//! muscle driverss aren't there, or they're just late.
//!
//! @param timeout_us How long, in microseconds, after the start of frame should we give up waiting?
//!
//! @author Hugo Elias
void Wait_For_All_Muscles_To_Send_Data(int32u timeout_us)
{
    do
    {
        collect_muscle_data_CAN_messages(CAN1);
        collect_muscle_data_CAN_messages(CAN2);
    }
    while (  (not_all_muscle_data_received())
          && (get_frame_time_us() < timeout_us)
          );
}



//! Wait until a certain time within the 1000us frame time.
//! This is a useful function for waiting until the last
//! moment to see if any more CAN messages arrive, before
//! writing data back to the ET1200.
//!
//! Frame time is in microseconds, starting at 0 when the EtherCAT
//! packet arrives, and should never be seen to go beyond 1000, when
//! the next EtherCAT packet should arrive.
//!
//! @param frame_time_us The frame time in microseconds.
//!
//! @author Hugo Elias
void Wait_For_Until_Frame_Time(int32u frame_time_us)
{
    while (get_frame_time_us() <= frame_time_us)
    {
    }
}



//! Idle time is the time the PIC spends waiting for the next EtherCAT packet to arrive.
//! The timer is started when the Status data has been completely written to the ET1200,
//! and stopped as soon as the PIC knows that new Command data are available.
//! 
//! @author Hugo Elias
int16u calculate_idle_time(void)
{
    return CORE_TIMER_TO_MICROSECONDS(frame_start_time - idle_time_start);
}




//! Read all of the Joint and Tactile sensors on the Palm
//!
//! @author Hugo Elias
void Read_All_Sensors(void)
{
    int8u adc_channel;
    int8u j=0;
    int32u frame_time=0;

    Nop();
    Nop();

    for (j=0; j<SENSORS_NUM_0320; j++)
        etherCAT_status_data.sensors[j] = j;

    switch (tactile_sensor_protocol)                                                // Read tactile sensors before joint sensors?
    {                                                                               // ------------------------------------------
        case TACTILE_SENSOR_PROTOCOL_TYPE_PST3:
            break;

        case TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3:                               // BioTac Pac needs to be read once before joint sensors.
            frame_time = get_frame_time_us();
            biotac_read_pac(0, &etherCAT_command_data, &etherCAT_status_data);
            break;

        default:
            break;
    }

    delay_us(10);


    j=0;
    for (adc_channel = 0; adc_channel < 8; ++adc_channel)                           // Now read joint sensors.
    {                                                                               // -----------------------
        switch (etherCAT_command_data.EDC_command)
        {
            case EDC_COMMAND_SENSOR_DATA: 
                SPIP_max_bits = 19;
                SPIP_Xceive_Fast(adc_channel);
                break;

            case EDC_COMMAND_SENSOR_CHANNEL_NUMBERS:
                SPIP_PutChannel_Numbers(adc_channel);
                break;

            default: 
                SPIP_PutChannel_Numbers(adc_channel);
                break;
        }

        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(0);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(1);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(2);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(3);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(4);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(5);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(6);
        etherCAT_status_data.sensors[ palm_EDC_0300_sensor_mapping[j++] ] = TRANSLATE_SOMI_MCP3208(7);
    }

    #if AUTO_TRIGGER == 1
        //etherCAT_command_data.tactile_data_type = TACTILE_SENSOR_TYPE_BIOTAC_PDC;
        //tactile_sensor_protocol                   = TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3;
    #endif

    delay_us(10);

    switch (tactile_sensor_protocol)                                                // Read tactile sensors after joint sensors?
    {                                                                               // -----------------------------------------

        case TACTILE_SENSOR_PROTOCOL_TYPE_PST3:                                     // PST3s are read after
            read_PSTs(&etherCAT_command_data, &etherCAT_status_data);
            break;

        case TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3:                               // BioTac: Hall, Misc, and Pac again are read after joint sensors.
            biotac_read_sensors(&etherCAT_command_data, &etherCAT_status_data, 0);

            Wait_For_All_Muscles_To_Send_Data(frame_time+490);                       // Use this delay time productively
            Wait_For_Until_Frame_Time(frame_time+500);                              // Now that we're close to the time, use a more accurate wait routine.

            biotac_read_pac(1, &etherCAT_command_data, &etherCAT_status_data);      // Read the Pac again at 500us to achieve 2000Hz.
            break;

        default:
            break;
    }


    if (etherCAT_command_data.tactile_data_type == TACTILE_SENSOR_TYPE_WHICH_SENSORS)   // Is the host asking which tactile sensor protocol we're using?
    {
        etherCAT_status_data.tactile[0].word[0] = tactile_sensor_protocol;                // Then tell it!
        etherCAT_status_data.tactile[1].word[0] = tactile_sensor_protocol;
        etherCAT_status_data.tactile[2].word[0] = tactile_sensor_protocol;
        etherCAT_status_data.tactile[3].word[0] = tactile_sensor_protocol;
        etherCAT_status_data.tactile[4].word[0] = tactile_sensor_protocol;
        etherCAT_status_data.tactile_data_valid = 0x001F;
        etherCAT_status_data.tactile_data_type = TACTILE_SENSOR_TYPE_WHICH_SENSORS;
    }
}



//! Zero the array containing the muscle data so that it's tidy.
//!
//! @author Hugo Elias
void zero_muscle_data_packets(void)
{
    int i, j;

    for (i=0; i<NUM_MUSCLE_DATA_PACKETS; i++)
    {
        for (j=0; j<8; j++)
        {
            etherCAT_status_data.muscle_data_packet[i].raw[j] = 0;
        }
    }


    //etherCAT_status_data.which_pressure_data_arrived[]i = 0x00;
    //etherCAT_status_data.which_muscle_data_had_errors = 0x00000000;
}



//! Ask the ET1200 if an EtherCAT packet arrived. If so, get it serviced.
//! Also, handle the LEDs etc if they need doing.
//! 
//! @author Hugo Elias
void Check_For_EtherCAT_Packet(void)
{
#warning FIX ME
    //collect_muscle_data_CAN_messages(CAN1);                                               // Mop up any missed CAN messages
    //collect_muscle_data_CAN_messages(CAN2);                                               // 

    #if AUTO_TRIGGER == 1                                                                   // This is just for debugging.
        if (get_frame_time_us() > 1150)                                                     // 
        {                                                                                   // 
            frame_start_time = ReadCoreTimer();                                             // 
            approx_1ms_handler();                                                           // 

            etherCAT_command_data.EDC_command    = EDC_COMMAND_SENSOR_DATA;      // Normal sensor sampling mode
            etherCAT_command_data.from_muscle_data_type = MUSCLE_DATA_PRESSURE;
            etherCAT_command_data.muscle_data[ 0] = 0xCD;
            etherCAT_command_data.muscle_data[ 1] = 0xEF;
            etherCAT_command_data.muscle_data[ 2] = 0x01;
            etherCAT_command_data.muscle_data[ 3] = 0x23;
            etherCAT_command_data.muscle_data[ 4] = 0xCD;
            etherCAT_command_data.muscle_data[ 5] = 0xEF;
            etherCAT_command_data.muscle_data[ 6] = 0x01;
            etherCAT_command_data.muscle_data[ 7] = 0x23;
            etherCAT_command_data.muscle_data[ 8] = 0xCD;
            etherCAT_command_data.muscle_data[ 9] = 0x01;
            etherCAT_command_data.muscle_data[10] = 0xCD;
            etherCAT_command_data.muscle_data[11] = 0xEF;
            etherCAT_command_data.muscle_data[12] = 0x01;
            etherCAT_command_data.muscle_data[13] = 0x23;
            etherCAT_command_data.muscle_data[14] = 0xCD;
            etherCAT_command_data.muscle_data[15] = 0xEF;
            etherCAT_command_data.muscle_data[16] = 0x01;
            etherCAT_command_data.muscle_data[17] = 0x23;
            etherCAT_command_data.muscle_data[18] = 0xCD;
            etherCAT_command_data.muscle_data[19] = 0x01;
            Service_EtherCAT_Packet();
        }                                                                                   // 
        return;                                                                             // 
    #endif                                                                                  // 


    ET1200_Update();

    if (There_is_Command_From_ET1200())
    {   
        frame_start_time = ReadCoreTimer();                                                 // The frame starts NOW!
        approx_1ms_handler();                                                               // Normally, the EtherCAT packets are used as a 1ms time base.
        Service_EtherCAT_Packet();
        return;
    }

    if (get_frame_time_us() > 1300)                                                         // If there are no EtherCAT packets,
    {                                                                                       // then we need to trigger approx_1ms_handler()
        frame_start_time = ReadCoreTimer();                                                 // And so we think of a frame starting now.
        approx_1ms_handler();                                                               // by time instead. 
        return;
    }
}



//! This is the main function of the whole node. It's called every time
//! an EtherCAT packet arrives.
//!
//! @author Hugo Elias
void Service_EtherCAT_Packet(void)
{
    #if AUTO_TRIGGER == 0
        Read_Commands_From_ET1200();                                                            // Read the command data
    #endif

    delay_us(20);                                                                           // Give the SPI port a little time to read the values we need now
                                                                                            // - EDC_command
                                                                                            // - from_motor_data_type


    switch (etherCAT_command_data.EDC_command)
    {
        case EDC_COMMAND_SENSOR_DATA:                                                       // Normal Command / Status packet
                                                                                            // ------------------------------

                                                                                            // This EDC_command is handled,
            LED_Off(ET1200_AL_err_LED);                                                     // so extinguish the Application Layer LED

            send_CAN_request_data_message( etherCAT_command_data.from_muscle_data_type);    // Start of frame message


            zero_muscle_data_packets();                                                     // Housekeeping

            Read_All_Sensors();                                                             // Read all joint and tactile sensors.
            num_muscle_CAN_messages_received_this_frame = 0;
            Wait_For_All_Muscles_To_Send_Data(700);                                         // Wait for 500us max.
            Send_Data_To_Muscles(&etherCAT_command_data);                                   // Send CAN messages to muscles

            etherCAT_status_data.EDC_command  = EDC_COMMAND_SENSOR_DATA;                    // FIXME: I don't think this calculation is correct
            etherCAT_status_data.idle_time_us = calculate_idle_time();                      // 

            #if AUTO_TRIGGER == 0
                write_status_data_To_ET1200();
            #endif
            idle_time_start = ReadCoreTimer();                                              // Idle time begins now because the status data
                                                                                            // has been written to the ET1200. The next EtherCAT
                                                                                            // packet can arrive any time after this.
            break;



        case EDC_COMMAND_CAN_DIRECT_MODE:                                                   // CAN Bridge packet
                                                                                            // -----------------

                                                                                            // This EDC_command is handled,
            LED_Off(ET1200_AL_err_LED);                                                     // so extinguish the Application Layer LED

            if ( ROS_Wants_me_to_send_CAN() )
            {
                read_ET1200_register_N(ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS, ETHERCAT_CAN_BRIDGE_DATA_SIZE, (int8u*)(&can_bridge_data_from_ROS));
                send_CAN_message_from_ROS();
                global_AL_Event_Register = read_ET1200_register_32u(0x220);
            }

            etherCAT_status_data.EDC_command  = EDC_COMMAND_SENSOR_DATA;
            etherCAT_status_data.idle_time_us = calculate_idle_time();                      // FIXME: I don't think this calculation is correct
            write_status_data_To_ET1200();                                                  // 

            Wait_For_Until_Frame_Time(600);                                                 // Give the node a little time to respond, and we might be able
                                                                                            // to get the reply message back into the very next EtherCAT packet.

            collect_one_CAN_message();                                                      // From either CAN bus

            write_ET1200_register_N(ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS, ETHERCAT_CAN_BRIDGE_DATA_SIZE, (int8u*)(&can_bridge_data_to_ROS));
            idle_time_start = ReadCoreTimer();                                              // Idle time begins now

            break;


        default:                                                                            // This EDC_command is not handled, and considered an error
            LED_On(ET1200_AL_err_LED);                                                      // So light the red Application Layer LED.
            break;
    }
}



//! Called when entering the Init state
//!
//! @author Hugo Elias (based on code by Rich Walker)
void Platform_Init_Callback(void)
{
    unsigned char i;

    etherCAT_status_data.EDC_command         = 0xCC;//EDC_COMMAND_SENSOR_DATA;
    etherCAT_command_data.EDC_command        = 0xDD;//EDC_COMMAND_SENSOR_DATA;
    can_bridge_data_from_ROS.message_data[0] = 0xff;

    for (i = 1; i < ETHERCAT_STATUS_DATA_SIZE / 2 ; ++i)
        ((int16s *)&etherCAT_status_data)[i] = 0xDEAD;
    
    write_ET1200_register_N  ( ETHERCAT_STATUS_DATA_ADDRESS, ETHERCAT_STATUS_DATA_SIZE,   (int8u*)&etherCAT_status_data);
}



//! Catch error codes generated by other functions.
//! Report those errors to the user.
//! Currently only handles CAN bus errors, which are reported by flickering the red LED for that bus.
//! 
//! @param error_code   an error code as defined in this_node.h
//! 
//! @author Hugo Elias
void report_error_code(ERROR_CODE  error_code)
{
    switch (error_code)
    {
        case CAN1_TX_NO_BUFFER:
        case CAN1_TX_BUFFER_FULL:
        case CAN1_RX_BUFFER_FULL:
        case CAN1_BECOME_SAD:
        case SIMPLE_CAN_BAD_MESSAGE_BUS_1:          
            LED_Single_Flicker(LED_CAN1_ERR);
            break;

        case CAN2_TX_NO_BUFFER:
        case CAN2_TX_BUFFER_FULL:
        case CAN2_RX_BUFFER_FULL:
        case CAN2_BECOME_SAD:
        case SIMPLE_CAN_BAD_MESSAGE_BUS_2:
            LED_Single_Flicker(LED_CAN2_ERR);
            break;

        default:
            NYI();                                  // Otherwise, panic!
            break;
    }
}



//! Catch event codes generated by other functions.
//! Report those events to the user.
//! Currently only handles CAN bus events, which are reported by flickering the blue LEDs for that bus.
//! 
//! @param event_code   an event code as defined in this_node.h
//! 
//! @author Hugo Elias
void report_event_code   (EVENT_CODE  event_code)
{
    switch (event_code)
    {
/*
        case CAN1_RECEIVED_MESSAGE:         LED_Single_Flicker(LED_CAN1_RX);            break;
        case CAN1_TRANSMITTED_MESSAGE:      LED_Single_Flicker(LED_CAN1_TX);            break;

        #ifdef LED_CAN2_RX_PIN
            case CAN2_RECEIVED_MESSAGE:         LED_Single_Flicker(LED_CAN2_RX);            break;
        #endif

        case CAN2_TRANSMITTED_MESSAGE:      LED_Single_Flicker(LED_CAN2_TX);            break;
*/

        case CAN1_RECEIVED_MESSAGE:         LED_On_For_5ms(LED_CAN1_RX);            break;
        case CAN1_TRANSMITTED_MESSAGE:      LED_On_For_5ms(LED_CAN1_TX);            break;

        #ifdef LED_CAN2_RX_PIN
            case CAN2_RECEIVED_MESSAGE:         LED_On_For_5ms(LED_CAN2_RX);            break;
        #endif

        case CAN2_TRANSMITTED_MESSAGE:      LED_On_For_5ms(LED_CAN2_TX);            break;

        default:
            break;
    }
}



//! Currently unimplemented. Will cause assert failure if called.
//! 
//! @author Hugo Elias
void report_error_string (char *error_string, ...)
{
    
    assert_dynamic(0);                  // Protection while function is as yet unimplemented
    error_string = NULL;
}



//! Currently unimplemented, but safe to call.
//! 
//! @author Hugo Elias
void report_event_string (char *event_string, ...)
{
    //assert_dynamic(0);                  // Protection while function is as yet unimplemented
    event_string = NULL; // to avoid warning
}



//! Flash all the lights like crazy to show that there has been an assert failure.
//! 
//! @author Hugo Elias
void __assert_dynamic_fail(const char* UNUSED(fmt), ...)
{
    unsigned int i;

    TRISDCLR = 0b00000100111110001;
    PORTDCLR = 0b00000100111110001;

    while(1)
    {
        PORTDINV = 0b00000100111110001;
    
        for (i=0; i<1000000; i++)
        {
            Nop();
            Nop();
            Nop();
            Nop();
        }
    }    
}



//! Called when a CAN message arrives from Muscle driver board, containing 5 measured pressure values, or some other data according to data_type
//! Declared in simple_can_muscle.h
//! 
//! @param data_type    Tells us the meaning of the values
//! @param muscle_set   Which set of muscles does this message come from?  range[0..7]. 
//!
//! @author Hugo Elias

void Received_Muscle_Data(FROM_MUSCLE_DATA_TYPE data_type, int8u muscle_data_packet_number, MUSCLE_DATA_PACKET *muscle_data_packet)
{
    
    if (muscle_data_packet_number > 7)                                                                       // A muscle_driver_number > 1 is an error
    {                                                                                                   // Let's just ignore it and hope it goes away.
        return;
    }

    FROM_MUSCLE_DATA_TYPE    expected_data_type   = etherCAT_command_data.from_muscle_data_type;

    num_muscle_CAN_messages_received_this_frame++;


    if (data_type == expected_data_type)                                                                // This is good :)
    {
        etherCAT_status_data.muscle_data_type = data_type;

        //muscle_driver_number *= NUM_PRESSURE_SENSORS_PER_MESSAGE;


        etherCAT_status_data.muscle_data_packet[muscle_data_packet_number] = *muscle_data_packet;                                                                                                       // Unpack the

        /*
        twelve_bit_int_to_nibbles.nibbles.padding = 0;

        twelve_bit_int_to_nibbles.nibbles.L = muscle_data_packet->packed.pressure0_L;
        twelve_bit_int_to_nibbles.nibbles.M = muscle_data_packet->packed.pressure0_M;
        twelve_bit_int_to_nibbles.nibbles.H = muscle_data_packet->packed.pressure0_H;
        etherCAT_status_data.pressure_sensors[muscle_driver_number + 0] = twelve_bit_int_to_nibbles.integer;

        twelve_bit_int_to_nibbles.nibbles.L = muscle_data_packet->packed.pressure1_L;
        twelve_bit_int_to_nibbles.nibbles.M = muscle_data_packet->packed.pressure1_M;
        twelve_bit_int_to_nibbles.nibbles.H = muscle_data_packet->packed.pressure1_H;
        etherCAT_status_data.pressure_sensors[muscle_driver_number + 1] = twelve_bit_int_to_nibbles.integer;

        twelve_bit_int_to_nibbles.nibbles.L = muscle_data_packet->packed.pressure2_L;
        twelve_bit_int_to_nibbles.nibbles.M = muscle_data_packet->packed.pressure2_M;
        twelve_bit_int_to_nibbles.nibbles.H = muscle_data_packet->packed.pressure2_H;
        etherCAT_status_data.pressure_sensors[muscle_driver_number + 2] = twelve_bit_int_to_nibbles.integer;

        twelve_bit_int_to_nibbles.nibbles.L = muscle_data_packet->packed.pressure3_L;
        twelve_bit_int_to_nibbles.nibbles.M = muscle_data_packet->packed.pressure3_M;
        twelve_bit_int_to_nibbles.nibbles.H = muscle_data_packet->packed.pressure3_H;
        etherCAT_status_data.pressure_sensors[muscle_driver_number + 3] = twelve_bit_int_to_nibbles.integer;

        twelve_bit_int_to_nibbles.nibbles.L = muscle_data_packet->packed.pressure4_L;
        twelve_bit_int_to_nibbles.nibbles.M = muscle_data_packet->packed.pressure4_M;
        twelve_bit_int_to_nibbles.nibbles.H = muscle_data_packet->packed.pressure4_H;
        etherCAT_status_data.pressure_sensors[muscle_driver_number + 4] = twelve_bit_int_to_nibbles.integer;
        */

        
/*
        etherCAT_status_data.pressure_sensors[muscle_set + 0] = muscle_data_packet->packed.pressure0;   // Can't use a loop here, because pressure0 etc
        etherCAT_status_data.pressure_sensors[muscle_set + 1] = muscle_data_packet->packed.pressure1;   // are actually 12-bit fields, and you can't
        etherCAT_status_data.pressure_sensors[muscle_set + 2] = muscle_data_packet->packed.pressure2;   // have an array of bit fields because you can't
        etherCAT_status_data.pressure_sensors[muscle_set + 3] = muscle_data_packet->packed.pressure3;   // have a pointer to them. The fields exist because
        etherCAT_status_data.pressure_sensors[muscle_set + 4] = muscle_data_packet->packed.pressure4;   // of CAN bus packet size limit, and this C code nicely
*/                                                                                                        // unpacks the data.

        etherCAT_status_data.which_muscle_data_arrived                  |= 0x00000001 << muscle_data_packet_number;   // Record that a message arrived
        //etherCAT_status_data.which_muscle_data_had_errors               &= 0xFFFFFFFE << muscle_data_packet_number;   // and that it was good (as far as we can tell)
    }
    else
    {
        etherCAT_status_data.which_muscle_data_arrived                  |= 0x00000001 << muscle_data_packet_number;   // Record that a message arrived
        //etherCAT_status_data.which_muscle_data_had_errors               |= 0x00000001 << muscle_data_packet_number;   // and that it was bad :(
    }
}



//! Correct length pause, which works by watching the CP0 COUNT register.
//! 
//! @param microseconds The number of microseconds to pause for
//!
//! @author Hugo Elias
void delay_us(int32u microseconds) 
{ 
    int32u start = ReadCoreTimer(); 
    int32u stop  = (microseconds-1) * (SYSTEM_FREQ_HZ/2000000);     // Subtract 1 to account for call overheads.
    
    stop += start;                                  //
    
    while (ReadCoreTimer() < stop)                  // wait till Count reaches the stop value
    {
    }
}



//! Correct length pause, which works by watching the CP0 COUNT register.
//! This code handles one rollover, so, as long as the delay isn't more than 107 seconds, you should be OK.
//!
//! @param milliseconds The number of milliseconds to pause for
//!
//! @author Hugo Elias
void delay_ms(int32u milliseconds)
{ 
    int32u start     = ReadCoreTimer(); 
    int32u num_ticks = milliseconds * (SYSTEM_FREQ_HZ/2000);
    
    while (ReadCoreTimer()-start < num_ticks)                  // wait till Count reaches the stop value
    {
    }
} 


