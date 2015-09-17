/**
 * @file   sr_edc.h
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

#ifndef SR_EDC_ETHERCAT_DRIVERS_SR_EDC_H
#define SR_EDC_ETHERCAT_DRIVERS_SR_EDC_H

#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <sr_edc_ethercat_drivers/sr0x.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <sr_robot_msgs/SimpleMotorFlasher.h>
#include <pthread.h>
#include <bfd.h>
#include <boost/smart_ptr.hpp>
#include <map>
#include <string>
#include <vector>
#include <boost/assign.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <sr_robot_msgs/EthercatDebug.h>

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/common/ethercat_can_bridge_protocol.h>
#include <sr_external_dependencies/external/common/common_edc_ethercat_protocol.h>
}


class SrEdc :
        public SR0X
{
public:
  SrEdc();

  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address, unsigned int ethercat_command_data_size,
                         unsigned int ethercat_status_data_size, unsigned int ethercat_can_bridge_data_size,
                         unsigned int ethercat_command_data_address, unsigned int ethercat_status_data_address,
                         unsigned int ethercat_can_bridge_data_command_address,
                         unsigned int ethercat_can_bridge_data_status_address);

  bool simple_motor_flasher(sr_robot_msgs::SimpleMotorFlasher::Request &req,
                            sr_robot_msgs::SimpleMotorFlasher::Response &res);

  void build_CAN_message(ETHERCAT_CAN_BRIDGE_DATA *message);

  bool can_data_is_ack(ETHERCAT_CAN_BRIDGE_DATA *packet);

  void erase_flash();

  // bool read_flash(unsigned int offset, unsigned char baddrl, unsigned char baddrh, unsigned char baddru);
  bool read_flash(unsigned int offset, unsigned int baddr);

protected:
  int counter_;
  ros::NodeHandle nodehandle_;
  ros::NodeHandle nh_tilde_;

  typedef realtime_tools::RealtimePublisher<std_msgs::Int16> rt_pub_int16_t;
  std::vector<boost::shared_ptr<rt_pub_int16_t> > realtime_pub_;

  /// Extra analog inputs real time publisher (+ accelerometer and gyroscope)
  boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > extra_analog_inputs_publisher;

  bool flashing;
  bool can_packet_acked;

  std::string device_id_;
  std::string device_joint_prefix_;

  /// This function will call the reinitialization function for the boards attached to the CAN bus
  virtual void reinitialize_boards() = 0;

  /**
   * Given the identifier for a certain board (motor board/ muscle driver) determines the right value
   * for the CAN bus and the ID of the board in that CAN bus.
   *
   * @param board_id the unique identifier for the board
   * @param can_bus pointer to the can bus number we want to determine
   * @param board_can_id pointer to the board id we want to determine
   */
  virtual void get_board_id_and_can_bus(int board_id, int *can_bus, unsigned int *board_can_id) = 0;

private:
  // static const unsigned int        nb_sensors_const;
  static const unsigned int max_retry;
  // static const unsigned short int  max_iter_const;
  // static const unsigned short int  ros_pub_freq_const;
  // static const unsigned short int  device_pub_freq_const;
  // static const unsigned char       nb_publish_by_unpack_const;
  // std::string                      firmware_file_name;
  pthread_mutex_t producing;
  ros::ServiceServer serviceServer;


  ETHERCAT_CAN_BRIDGE_DATA can_message_;
  bool can_message_sent;
  bfd_byte *binary_content;  // buffer containing the binary content to be flashed
  unsigned int pos;  // position in binary_content buffer
  unsigned int motor_being_flashed;


  // We're using 2 can busses, so can_bus_ is 1 for motors 0 to 9 and 2 for motors 10 to 19
  int can_bus_;

  /**
   * Contains the common procedure to send a CAN message (i.e. write it in the global struct from which it is read, added to the
   * next ethercat frame, and sent) and wait for the can_packet_acked flag to be set
   *
   * @param can_bus which of the 2 CAN buses (0 or 1) wil be used
   * @param msg_id id of the CAN message
   * @param msg_length the length in bytes of msg_data
   * @param msg_data data of the CAN message
   * @param timeout time max (in ms) to wait for the can_packet_acked flag to be set
   * @param timedout if true, the can_packet_acked flag wasn't set before the timeout
   */
  void send_CAN_msg(int8u can_bus, int16u msg_id, int8u msg_length, int8u msg_data[], int timeout, bool *timedout);

  /**
   * Read back the firmware from the flash of the PIC, and checks it against the data read from the object (.hex) file
   *
   * @param baddr the base address of the code (the lowest address to be written on the flash)
   * @param total_size the size in bytes of the code to write
   *
   * @return true if both are identical
   */
  bool read_back_and_check_flash(unsigned int baddr, unsigned int total_size);

  /**
   * Look for the start and end address of every section in the hex file,
   * to detect the lowest and highest address of the data we need to write in the PIC's flash.
   * The sections starting at an address higher than 0x7fff will be ignored as they are not proper "code memory" firmware
   * (they can contain the CONFIG bits of the microcontroller, which we don't want to write here)
   * To understand the structure (sections) of the object file containing the firmware (usually a .hex) the following commands can be useful:
   *   \code objdump -x simplemotor.hex \endcode
   *   \code objdump -s simplemotor.hex \endcode
   *
   * @param fd pointer to a bfd file structure
   * @param smallest_start_address the lowest address found is returned through this pointer
   * @param biggest_end_address the highest address found is returned through this pointer
   */
  void find_address_range(bfd *fd, unsigned int *smallest_start_address, unsigned int *biggest_end_address);

  /**
   * Reads the content from the object (.hex) file and stores it in a previously reserved memory space
   *
   * @param fd pointer to a bfd file structure
   * @param content a pointer to the memory space where we want to store the firmware
   * @param base_addr the base address of the code (the lowest address to be written on the flash)
   *
   * @return true if the reading succeeds
   */
  bool read_content_from_object_file(bfd *fd, bfd_byte *content, unsigned int base_addr);

  /**
   * Writes the code previously read from the hex file to the flash memory of the PIC
   *
   * @param base_addr the base address of the code (the lowest address to be written on the flash)
   * @param total_size the size in bytes of the code to write
   *
   * @return true if the writing process succeeds
   */
  bool write_flash_data(unsigned int base_addr, unsigned int total_size);

  /**
   * Extract the filename from the full path.
   *
   * @param full_path The full path.
   *
   * @return the filename.
   */
  static inline std::string get_filename(std::string full_path)
  {
    std::vector<std::string> splitted_string;
    boost::split(splitted_string, full_path, boost::is_any_of("/"));
    return splitted_string.back();
  }
};


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif  // SR_EDC_ETHERCAT_DRIVERS_SR_EDC_H
