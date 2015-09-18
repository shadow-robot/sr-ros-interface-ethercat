/**
 * @file   sr06.h
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
 *         Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
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
 * @brief This is a ROS driver for Shadow Robot #6 EtherCAT product ID
 *
 *
 */

#ifndef SR_EDC_ETHERCAT_DRIVERS_SR06_H
#define SR_EDC_ETHERCAT_DRIVERS_SR06_H

#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <sr_edc_ethercat_drivers/sr_edc.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <sr_robot_msgs/SimpleMotorFlasher.h>
#include <pthread.h>
#include <bfd.h>
#include <boost/smart_ptr.hpp>
#include <map>
#include <vector>
#include <boost/assign.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>

#include <sr_robot_lib/sr_motor_hand_lib.hpp>

#include <sr_robot_msgs/EthercatDebug.h>

#include <sr_external_dependencies/types_for_external.h>

extern "C"
{
#include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
}

class SR06 :
        public SrEdc
{
public:
  SR06();

  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);

  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed = true);

  virtual void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer);

  virtual void packCommand(unsigned char *buffer, bool halt, bool reset);

  virtual bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

protected:
  typedef realtime_tools::RealtimePublisher<std_msgs::Int16> rt_pub_int16_t;
  std::vector<boost::shared_ptr<rt_pub_int16_t> > realtime_pub_;

  /// Extra analog inputs real time publisher (+ accelerometer and gyroscope)
  boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > extra_analog_inputs_publisher;

  /// This function will call the reinitialization function for the boards attached to the CAN bus
  virtual void reinitialize_boards();

  /**
   * Given the identifier for a certain board (motor board/ muscle driver) determines the right value
   * for the CAN bus and the ID of the board in that CAN bus.
   *
   * @param board_id the unique identifier for the board
   * @param can_bus pointer to the can bus number we want to determine
   * @param board_can_id pointer to the board id we want to determine
   */
  virtual void get_board_id_and_can_bus(int board_id, int *can_bus, unsigned int *board_can_id);

private:
  // std::string                      firmware_file_name;

  // counter for the number of empty buffer we're reading.
  unsigned int zero_buffer_read;

  boost::shared_ptr<shadow_robot::SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS,
          ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND> > sr_hand_lib;

  /**
   *a counter used to publish the tactiles at 100Hz:
   * count 10 cycles, then reset the cycle_count to 0.
   */
  int16_t cycle_count;

  /// Debug real time publisher: publishes the raw ethercat data
  boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::EthercatDebug> > debug_publisher;
};


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


#endif  // SR_EDC_ETHERCAT_DRIVERS_SR06_H

