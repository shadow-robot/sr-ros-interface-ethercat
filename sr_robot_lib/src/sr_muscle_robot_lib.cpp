/**
 * @file   sr_muscle_robot_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
 * @date   Tue Mar  19 17:12:13 2013
 *
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
 *
 * @brief This is a generic robot library for Shadow Robot's muscle-actuated Hardware.
 *
 *
 */

#include "sr_robot_lib/sr_muscle_robot_lib.hpp"
#include <string>
#include <utility>
#include <map>
#include <vector>
#include <boost/foreach.hpp>

#include <sys/time.h>

#include <ros/ros.h>

#include <controller_manager_msgs/ListControllers.h>

#define SERIOUS_ERROR_FLAGS PALM_0300_EDC_SERIOUS_ERROR_FLAGS
#define error_flag_names palm_0300_edc_error_flag_names


using std::vector;
using std::string;
using std::pair;
using std::map;
using std::ostringstream;
using sr_actuator::SrMuscleActuator;
using shadow_joints::CalibrationMap;
using shadow_joints::Joint;
using shadow_joints::JointToSensor;
using shadow_joints::JointToMuscle;
using shadow_joints::MuscleWrapper;
using shadow_joints::MuscleDriver;
using shadow_joints::PartialJointToSensor;
using generic_updater::MuscleUpdater;
using boost::shared_ptr;
using boost::static_pointer_cast;
using boost::shared_ptr;

namespace shadow_robot
{
  template<class StatusType, class CommandType>
  const double SrMuscleRobotLib<StatusType, CommandType>::timeout = 5.0;

  template<class StatusType, class CommandType>
  SrMuscleRobotLib<StatusType, CommandType>::SrMuscleRobotLib(hardware_interface::HardwareInterface *hw,
                                                              ros::NodeHandle nh, ros::NodeHandle nhtilde,
                                                              string device_id, string joint_prefix)
          : SrRobotLib<StatusType, CommandType>(hw, nh, nhtilde, device_id, joint_prefix),
            muscle_current_state(operation_mode::device_update_state::INITIALIZATION), init_max_duration(timeout),
            lock_init_timeout_(shared_ptr<boost::mutex>(new boost::mutex())),

            // Create a one-shot timer
            check_init_timeout_timer(this->nh_tilde.createTimer(init_max_duration,
                                                                boost::bind(
                                                                        &SrMuscleRobotLib<StatusType,
                                                                                CommandType>::init_timer_callback,
                                                                        this, _1), true)),
            pressure_calibration_map_(read_pressure_calibration())
  {
#ifdef DEBUG_PUBLISHER
    this->debug_muscle_indexes_and_data.resize(this->nb_debug_publishers_const);
    for (int i = 0; i < this->nb_debug_publishers_const; ++i)
    {
      ostringstream ss;
      ss << "srh/debug_" << i;
      this->debug_publishers.push_back(this->node_handle.template advertise<std_msgs::Int16>(ss.str().c_str(), 100));
    }
#endif
  }

  template<class StatusType, class CommandType>
  CalibrationMap SrMuscleRobotLib<StatusType, CommandType>::read_pressure_calibration()
  {
    ROS_INFO("Reading pressure calibration");
    CalibrationMap pressure_calibration;
    string param_name = "sr_pressure_calibrations";

    XmlRpc::XmlRpcValue calib;
    this->nodehandle_.getParam(param_name, calib);
    ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // iterate on all the joints
    for (int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      // check the calibration is well formatted:
      // first joint name, then calibration table
      ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

      string joint_name = static_cast<string> (calib[index_cal][0]);
      vector<joint_calibration::Point> calib_table_tmp;

      // now iterates on the calibration table for the current joint
      for (int32_t index_table = 0; index_table < calib[index_cal][1].size(); ++index_table)
      {
        ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
        // only 2 values per calibration point: raw and calibrated (doubles)
        ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
        ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


        joint_calibration::Point point_tmp;
        point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
        point_tmp.calibrated_value = static_cast<double> (calib[index_cal][1][index_table][1]);
        calib_table_tmp.push_back(point_tmp);
      }

      pressure_calibration.insert(joint_name, shared_ptr<shadow_robot::JointCalibration>(
              new shadow_robot::JointCalibration(calib_table_tmp)));
    }

    return pressure_calibration;
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::update(StatusType *status_data)
  {
    // read the PIC idle time
    this->main_pic_idle_time = status_data->idle_time_us;
    if (status_data->idle_time_us < this->main_pic_idle_time_min)
    {
      this->main_pic_idle_time_min = status_data->idle_time_us;
    }

    // get the current timestamp
    struct timeval tv;
    double timestamp = 0.0;
    if (gettimeofday(&tv, NULL))
    {
      ROS_WARN("SrMuscleRobotLib: Failed to get system time, timestamp in state will be zero");
    }
    else
    {
      timestamp = static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) / 1.0e+6;
    }

    // First we read the tactile sensors information
    this->update_tactile_info(status_data);

    // then we read the muscle drivers information
    for (vector<MuscleDriver>::iterator muscle_driver_tmp = this->muscle_drivers_vector_.begin();
         muscle_driver_tmp != this->muscle_drivers_vector_.end();
         ++muscle_driver_tmp)
    {
      read_muscle_driver_data(muscle_driver_tmp, status_data);
    }

    // then we read the joints informations
    for (vector<Joint>::iterator joint_tmp = this->joints_vector.begin();
         joint_tmp != this->joints_vector.end();
         ++joint_tmp)
    {
      if (!joint_tmp->has_actuator)
      {
        continue;
      }

      SrMuscleActuator *actuator = this->get_joint_actuator(joint_tmp);
      shared_ptr<MuscleWrapper> muscle_wrapper = static_pointer_cast<MuscleWrapper>(joint_tmp->actuator_wrapper);

      // Fill in the tactiles.
      if (this->tactiles != NULL)
      {
        actuator->muscle_state_.tactiles_ = this->tactiles->get_tactile_data();
      }

      this->process_position_sensor_data(joint_tmp, status_data, timestamp);

      // if no muscle is associated to this joint, then continue
      if ((muscle_wrapper->muscle_driver_id[0] == -1))
      {
        continue;
      }

      read_additional_muscle_data(joint_tmp, status_data);
    }  // end for joint
  }  // end update()

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::build_command(CommandType *command)
  {
    if (muscle_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      muscle_current_state = muscle_updater_->build_init_command(command);
    }
    else
    {
      // build the muscle command
      muscle_current_state = muscle_updater_->build_command(command);
    }

    // Build the tactile sensors command
    this->build_tactile_command(command);

    ///////
    // Now we chose the command to send to the muscle
    // by default we send a valve demand, but if we have a reset command,
    // then we send the reset.
    if (reset_muscle_driver_queue.empty())
    {
      command->to_muscle_data_type = MUSCLE_DEMAND_VALVES;

      // loop on all the joints and update their muscle: we're sending commands to all the muscles.
      for (vector<Joint>::iterator joint_tmp = this->joints_vector.begin();
           joint_tmp != this->joints_vector.end();
           ++joint_tmp)
      {
        if (joint_tmp->has_actuator)
        {
          shared_ptr<MuscleWrapper> muscle_wrapper = static_pointer_cast<MuscleWrapper>(joint_tmp->actuator_wrapper);
          SrMuscleActuator *muscle_actuator = static_cast<SrMuscleActuator *> (muscle_wrapper->actuator);

          unsigned int muscle_driver_id_0 = muscle_wrapper->muscle_driver_id[0];
          unsigned int muscle_driver_id_1 = muscle_wrapper->muscle_driver_id[1];
          unsigned int muscle_id_0 = muscle_wrapper->muscle_id[0];
          unsigned int muscle_id_1 = muscle_wrapper->muscle_id[1];

          if (!this->nullify_demand_)
          {
            set_valve_demand(&(command->muscle_data[(muscle_driver_id_0 * 10 + muscle_id_0) / 2]),
                             muscle_actuator->muscle_command_.valve_[0], ((uint8_t) muscle_id_0) & 0x01);
            set_valve_demand(&(command->muscle_data[(muscle_driver_id_1 * 10 + muscle_id_1) / 2]),
                             muscle_actuator->muscle_command_.valve_[1], ((uint8_t) muscle_id_1) & 0x01);

            muscle_actuator->muscle_state_.last_commanded_valve_[0] = muscle_actuator->muscle_command_.valve_[0];
            muscle_actuator->muscle_state_.last_commanded_valve_[1] = muscle_actuator->muscle_command_.valve_[1];
          }
          else
          {
            // We want to send a demand of 0
            set_valve_demand(&(command->muscle_data[(muscle_driver_id_0 * 10 + muscle_id_0) / 2]), 0,
                             ((uint8_t) muscle_id_0) & 0x01);
            set_valve_demand(&(command->muscle_data[(muscle_driver_id_1 * 10 + muscle_id_1) / 2]), 0,
                             ((uint8_t) muscle_id_1) & 0x01);

            muscle_actuator->muscle_state_.last_commanded_valve_[0] = 0;
            muscle_actuator->muscle_state_.last_commanded_valve_[1] = 0;
          }

#ifdef DEBUG_PUBLISHER
           // publish the debug values for the given muscles.
           // NB: debug_muscle_indexes_and_data is smaller
           //     than debug_publishers.
           int publisher_index = 0;
           shared_ptr<pair<int, int> > debug_pair;
           if (this->debug_mutex.try_lock())
           {
             BOOST_FOREACH(debug_pair, this->debug_muscle_indexes_and_data)
             {
               if (debug_pair != NULL)
               {
                 MuscleWrapper* actuator_wrapper = static_cast<MuscleWrapper*> (joint_tmp->actuator_wrapper.get());
                 // check if we want to publish some data for the current muscle
                 if (debug_pair->first == actuator_wrapper->muscle_id[0])
                 {
                   // check if it's the correct data
                   if (debug_pair->second == -1)
                   {
                     this->msg_debug.data = joint_tmp->actuator_wrapper->actuator->command_.effort_;
                     this->debug_publishers[publisher_index].publish(this->msg_debug);
                   }
                 }
               }
               publisher_index++;
             }

             this->debug_mutex.unlock();
           }  // end try_lock
#endif
        }  // end if has_actuator
      }  // end for each joint
    }  // endif
    else
    {
      // we have some reset command waiting.
      // We'll send all of them
      command->to_muscle_data_type = MUSCLE_SYSTEM_RESET;

      while (!reset_muscle_driver_queue.empty())
      {
        int16_t muscle_driver_id = reset_muscle_driver_queue.front();
        reset_muscle_driver_queue.pop();

        // reset the CAN messages counters for the muscle driver we're going to reset.
        for (vector<MuscleDriver>::iterator driver = this->muscle_drivers_vector_.begin();
             driver != this->muscle_drivers_vector_.end();
             ++driver)
        {
          if (driver->muscle_driver_id == muscle_driver_id)
          {
            driver->can_msgs_transmitted_ = 0;
            driver->can_msgs_received_ = 0;
          }
        }

        // we send the MUSCLE_SYSTEM_RESET_KEY
        // and the muscle_driver_id (on the bus)
        crc_unions::union16 to_send;
        to_send.byte[1] = MUSCLE_SYSTEM_RESET_KEY >> 8;
        if (muscle_driver_id > 1)
        {
          to_send.byte[0] = muscle_driver_id - 2;
        }
        else
        {
          to_send.byte[0] = muscle_driver_id;
        }

        command->muscle_data[muscle_driver_id * 5] = to_send.byte[0];
        command->muscle_data[muscle_driver_id * 5 + 1] = to_send.byte[1];
      }
    }  // end if reset queue not empty
  }

  template<class StatusType, class CommandType>
  inline void SrMuscleRobotLib<StatusType, CommandType>::set_valve_demand(uint8_t *muscle_data_byte_to_set,
                                                                          int8_t valve_value, uint8_t shift)
  {
    uint8_t tmp_valve = 0;

    // The encoding we want for the negative integers is represented in two's complement,
    // but based on a 4 bit data size instead of 8 bit, so
    // we'll have to do it manually
    if (valve_value < 0)
    {
      // Take the value as positive
      tmp_valve = -valve_value;
      // Do a bitwise inversion on the 4 lowest bytes only
      tmp_valve = (~tmp_valve) & 0x0F;
      // Add one
      tmp_valve = tmp_valve + 1;
    }
    else  // Positive representation is straightforward
    {
      tmp_valve = valve_value & 0x0F;
    }

    // A shift of 0 means that we want to write the value on the 4 least significant bits
    // shift of 1 means that we want to write the value on the 4 most significant bits

    // We zero the 4 bits that we want to write our valve value on
    *muscle_data_byte_to_set &= (0xF0 >> (shift * 4));
    // We write our valve value on those 4 bits
    *muscle_data_byte_to_set |= (tmp_valve << (shift * 4));
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::add_diagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                                                  diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for (vector<Joint>::iterator joint = this->joints_vector.begin();
         joint != this->joints_vector.end();
         ++joint)
    {
      ostringstream name("");
      string prefix = this->device_id_.empty() ? this->device_id_ : (this->device_id_ + " ");
      name << prefix << "SRDMuscle " << joint->joint_name;
      d.name = name.str();

      if (joint->has_actuator)
      {
        shared_ptr<MuscleWrapper> actuator_wrapper = static_pointer_cast<MuscleWrapper>(joint->actuator_wrapper);
        SrMuscleActuator *actuator = get_joint_actuator(joint);

        if (actuator_wrapper->actuator_ok)
        {
          if (actuator_wrapper->bad_data)
          {
            d.summary(d.WARN, "WARNING, bad CAN data received");

            d.clear();
            d.addf("Muscle ID 0", "%d", actuator_wrapper->muscle_id[0]);
            d.addf("Muscle driver ID 0", "%d", actuator_wrapper->muscle_driver_id[0]);
            d.addf("Muscle ID 1", "%d", actuator_wrapper->muscle_id[1]);
            d.addf("Muscle driver ID 1", "%d", actuator_wrapper->muscle_driver_id[1]);
          }
          else  // the data is good
          {
            d.summary(d.OK, "OK");

            d.clear();
            d.addf("Muscle ID 0", "%d", actuator_wrapper->muscle_id[0]);
            d.addf("Muscle driver ID 0", "%d", actuator_wrapper->muscle_driver_id[0]);
            d.addf("Muscle ID 1", "%d", actuator_wrapper->muscle_id[1]);
            d.addf("Muscle driver ID 1", "%d", actuator_wrapper->muscle_driver_id[1]);

            d.addf("Unfiltered position", "%f", actuator->muscle_state_.position_unfiltered_);

            d.addf("Last Commanded Valve 0", "%d", actuator->muscle_state_.last_commanded_valve_[0]);
            d.addf("Last Commanded Valve 1", "%d", actuator->muscle_state_.last_commanded_valve_[1]);

            d.addf("Unfiltered Pressure 0", "%u", actuator->muscle_state_.pressure_[0]);
            d.addf("Unfiltered Pressure 1", "%u", actuator->muscle_state_.pressure_[1]);

            d.addf("Position", "%f", actuator->state_.position_);
          }
        }
        else
        {
          d.summary(d.ERROR, "Muscle error");
          d.clear();
          d.addf("Muscle ID 0", "%d", actuator_wrapper->muscle_id[0]);
          d.addf("Muscle driver ID 0", "%d", actuator_wrapper->muscle_driver_id[0]);
          d.addf("Muscle ID 1", "%d", actuator_wrapper->muscle_id[1]);
          d.addf("Muscle driver ID 1", "%d", actuator_wrapper->muscle_driver_id[1]);
        }
      }
      else
      {
        d.summary(d.OK, "No muscle associated to this joint");
        d.clear();
      }
      vec.push_back(d);
    }  // end for each joints

    for (vector<MuscleDriver>::iterator muscle_driver = this->muscle_drivers_vector_.begin();
         muscle_driver != this->muscle_drivers_vector_.end();
         ++muscle_driver)
    {
      ostringstream name("");
      name << "Muscle driver " << muscle_driver->muscle_driver_id;
      d.name = name.str();

      if (muscle_driver->driver_ok)
      {
        if (muscle_driver->bad_data)
        {
          d.summary(d.WARN, "WARNING, bad CAN data received");

          d.clear();
          d.addf("Muscle Driver ID", "%d", muscle_driver->muscle_driver_id);
        }
        else  // the data is good
        {
          d.summary(d.OK, "OK");

          d.clear();
          d.addf("Muscle Driver ID", "%d", muscle_driver->muscle_driver_id);
          d.addf("Serial Number", "%d", muscle_driver->serial_number);
          d.addf("Assembly date", "%d / %d / %d", muscle_driver->assembly_date_day, muscle_driver->assembly_date_month,
                 muscle_driver->assembly_date_year);

          d.addf("Number of CAN messages received", "%lld", muscle_driver->can_msgs_received_);
          d.addf("Number of CAN messages transmitted", "%lld", muscle_driver->can_msgs_transmitted_);

          if (muscle_driver->firmware_modified_)
          {
            d.addf("Firmware git revision (server / pic / modified)", "%d / %d / True",
                   muscle_driver->server_firmware_git_revision_, muscle_driver->pic_firmware_git_revision_);
          }
          else
          {
            d.addf("Firmware git revision (server / pic / modified)", "%d / %d / False",
                   muscle_driver->server_firmware_git_revision_, muscle_driver->pic_firmware_git_revision_);
          }
        }
      }
      else
      {
        d.summary(d.ERROR, "Muscle Driver error");
        d.clear();
        d.addf("Muscle Driver ID", "%d", muscle_driver->muscle_driver_id);
      }

      vec.push_back(d);
    }  // end for each muscle driver
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::read_additional_muscle_data(vector<Joint>::iterator joint_tmp,
                                                                              StatusType *status_data)
  {
    if (!joint_tmp->has_actuator)
    {
      return;
    }

    int packet_offset_muscle_0 = 0;
    int packet_offset_muscle_1 = 0;

    shared_ptr<MuscleWrapper> muscle_wrapper = static_pointer_cast<MuscleWrapper>(joint_tmp->actuator_wrapper);

    // Every muscle driver sends two muscle_data_packet containing pressures from 5 muscles each
    if (muscle_wrapper->muscle_id[0] >= NUM_PRESSURE_SENSORS_PER_MESSAGE)
    {
      packet_offset_muscle_0 = 1;
    }

    if (muscle_wrapper->muscle_id[1] >= NUM_PRESSURE_SENSORS_PER_MESSAGE)
    {
      packet_offset_muscle_1 = 1;
    }

    // check the masks to see if the CAN messages arrived from the muscle driver
    // the flag should be set to 1 for each muscle driver CAN message
    // (a muscle driver sends 2 separate CAN messages with 5 muscle pressures each.
    // Every actuator (every joint) has two muscles, so we check both flags to decide that the actuator is OK
    muscle_wrapper->actuator_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_muscle_data_arrived,
                                                                        muscle_wrapper->muscle_driver_id[0] * 2 +
                                                                        packet_offset_muscle_0)
                                  && sr_math_utils::is_bit_mask_index_true(status_data->which_muscle_data_arrived,
                                                                           muscle_wrapper->muscle_driver_id[1] * 2 +
                                                                           packet_offset_muscle_1);

    /*
    // check the masks to see if a bad CAN message arrived
    // the flag should be 0
    muscle_wrapper->bad_data =
            sr_math_utils::is_bit_mask_index_true(status_data->which_pressure_data_had_errors,
                                                  muscle_wrapper->muscle_driver_id[0] * 10 +
                                                          muscle_wrapper->muscle_id[0]) &&
                    sr_math_utils::is_bit_mask_index_true(status_data->which_pressure_data_had_errors,
                                                          muscle_wrapper->muscle_driver_id[1] * 10 +
                                                                  muscle_wrapper->muscle_id[1]);
     */

    if (muscle_wrapper->actuator_ok)
    {
      SrMuscleActuator *actuator = static_cast<SrMuscleActuator *> (joint_tmp->actuator_wrapper->actuator);
      MuscleWrapper *actuator_wrapper = static_cast<MuscleWrapper *> (joint_tmp->actuator_wrapper.get());

#ifdef DEBUG_PUBLISHER
      int publisher_index = 0;
      // publish the debug values for the given muscles.
      // NB: debug_muscle_indexes_and_data is smaller
      //     than debug_publishers.
      shared_ptr<pair<int, int> > debug_pair;

      if (this->debug_mutex.try_lock())
      {
        BOOST_FOREACH(debug_pair, this->debug_muscle_indexes_and_data)
        {
          if (debug_pair != NULL)
          {
            // check if we want to publish some data for the current muscle
            if (debug_pair->first == actuator_wrapper->muscle_id[0])
            {
              // if < 0, then we're not asking for a FROM_MOTOR_DATA_TYPE
              if (debug_pair->second > 0)
              {
                // check if it's the correct data
                if (debug_pair->second == status_data->muscle_data_type)
                {
                  // @todo do something meaningful here
                  // this->msg_debug.data = status_data->muscle_data_packet[0].misc;
                  this->msg_debug.data = 0;
                  this->debug_publishers[publisher_index].publish(this->msg_debug);
                }
              }
            }
          }
          publisher_index++;
        }

        this->debug_mutex.unlock();
      }  // end try_lock
#endif

      // we received the data and it was correct
      unsigned int p1 = 0;
      switch (status_data->muscle_data_type)
      {
        case MUSCLE_DATA_PRESSURE:
          // Calibrate the raw values to bar pressure values.
          for (int i = 0; i < 2; ++i)
          {
            p1 = get_muscle_pressure(muscle_wrapper->muscle_driver_id[i], muscle_wrapper->muscle_id[i], status_data);
            string name = joint_tmp->joint_name + "_" + boost::lexical_cast<string>(i);
            // XXX: If the joint isn't found we crash
            // ROS_INFO_STREAM("Calib: "<<name);
            pressure_calibration_tmp_ = pressure_calibration_map_.find(name);
            double bar = pressure_calibration_tmp_->compute(static_cast<double> (p1));
            if (bar < 0.0)
            {
              bar = 0.0;
            }
            actuator->muscle_state_.pressure_[i] = static_cast<int16u> (bar);
          }
          // Raw values
          // actuator->state_.pressure_[0] = static_cast<int16u>(
          //         get_muscle_pressure(muscle_wrapper->muscle_driver_id[0],
          //                             muscle_wrapper->muscle_id[0],
          //                             status_data));
          // actuator->state_.pressure_[1] = static_cast<int16u>(
          //         get_muscle_pressure(muscle_wrapper->muscle_driver_id[1],
          //                             muscle_wrapper->muscle_id[1],
          //                             status_data));
          // ROS_WARN("DriverID: %u MuscleID: %u Pressure 0: %u", muscle_wrapper->muscle_driver_id[0],
          //          muscle_wrapper->muscle_id[0], actuator->state_.pressure_[0]);
          // ROS_WARN("DriverID: %u MuscleID: %u Pressure 1: %u", muscle_wrapper->muscle_driver_id[1],
          //          muscle_wrapper->muscle_id[1], actuator->state_.pressure_[1]);

#ifdef DEBUG_PUBLISHER
        if (actuator_wrapper->muscle_id[0] == 8)
        {
         // ROS_ERROR_STREAM("SGL " <<actuator->state_.strain_gauge_left_);
         // this->msg_debug.data = actuator->state_.strain_gauge_left_;
         // this->debug_publishers[0].publish(this->msg_debug);
        }
#endif
          break;


        default:
          break;
      }
    }
  }

  template<class StatusType, class CommandType>
  unsigned int SrMuscleRobotLib<StatusType, CommandType>::get_muscle_pressure(int muscle_driver_id, int muscle_id,
                                                                              StatusType *status_data)
  {
    unsigned int muscle_pressure = 0;
    int packet_offset = 0;
    int muscle_index = muscle_id;

    // Every muscle driver sends two muscle_data_packet containing pressures from 5 muscles each
    if (muscle_id >= NUM_PRESSURE_SENSORS_PER_MESSAGE)
    {
      packet_offset = 1;
      muscle_index = muscle_id - NUM_PRESSURE_SENSORS_PER_MESSAGE;
    }

    switch (muscle_index)
    {
      case 0:
        muscle_pressure =
                (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure0_H << 8)
                + (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure0_M << 4)
                + status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure0_L;
        break;

      case 1:
        muscle_pressure =
                (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure1_H << 8)
                + (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure1_M << 4)
                + status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure1_L;
        break;

      case 2:
        muscle_pressure =
                (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure2_H << 8)
                + (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure2_M << 4)
                + status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure2_L;
        break;

      case 3:
        muscle_pressure =
                (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure3_H << 8)
                + (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure3_M << 4)
                + status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure3_L;
        break;

      case 4:
        muscle_pressure =
                (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure4_H << 8)
                + (status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure4_M << 4)
                + status_data->muscle_data_packet[muscle_driver_id * 2 + packet_offset].packed.pressure4_L;
        break;

      default:
        ROS_ERROR("Incorrect muscle index: %d", muscle_index);
        break;
    }

    return muscle_pressure;
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::read_muscle_driver_data(
          vector<MuscleDriver>::iterator muscle_driver_tmp, StatusType *status_data)
  {
    // check one the masks (e.g. the first) for this muscle driver to see if
    // the CAN messages arrived correctly from the muscle driver
    // the flag should be set to 1 for each message in this driver.
    muscle_driver_tmp->driver_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_muscle_data_arrived,
                                                                         muscle_driver_tmp->muscle_driver_id * 2);
    /*
   // check the masks to see if a bad CAN message arrived
   // the flag should be 0
    muscle_driver_tmp->bad_data = sr_math_utils::is_bit_mask_index_true(status_data->which_pressure_data_had_errors,
                                                                        muscle_driver_tmp->muscle_driver_id * 2 );
     */

    if (muscle_driver_tmp->driver_ok)
    {
      // we received the data and it was correct
      set_muscle_driver_data_received_flags(status_data->muscle_data_type, muscle_driver_tmp->muscle_driver_id);

      switch (status_data->muscle_data_type)
      {
        case MUSCLE_DATA_PRESSURE:
          // We don't do anything here. This will be treated in a per-joint loop in read_additional_muscle_data
          break;

        case MUSCLE_DATA_CAN_STATS:
          // For the moment all the CAN data statistics for a muscle driver are contained
          // in the first packet coming from that driver
          // these are 16 bits values and will overflow -> we compute the real value.
          // This needs to be updated faster than the overflowing period (which should be roughly every 30s)
          muscle_driver_tmp->can_msgs_received_ = sr_math_utils::counter_with_overflow(
                  muscle_driver_tmp->can_msgs_received_,
                  static_cast<int16u> (status_data->muscle_data_packet[muscle_driver_tmp->muscle_driver_id *
                                                                       2].misc.can_msgs_rx));
          muscle_driver_tmp->can_msgs_transmitted_ = sr_math_utils::counter_with_overflow(
                  muscle_driver_tmp->can_msgs_transmitted_,
                  static_cast<int16u> (status_data->muscle_data_packet[muscle_driver_tmp->muscle_driver_id *
                                                                       2].misc.can_msgs_tx));

          // CAN bus errors
          muscle_driver_tmp->can_err_rx = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2].misc.can_err_rx);
          muscle_driver_tmp->can_err_tx = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2].misc.can_err_tx);
          break;

        case MUSCLE_DATA_SLOW_MISC:
          // We received a slow data:
          // the slow data type is not transmitted anymore (as it was in the muscle hand protocol)
          // Instead, all the information (of every data type) is contained in the 2 packets
          // that come from every muscle driver
          // So in fact this message is not "slow" anymore.
          muscle_driver_tmp->pic_firmware_git_revision_ = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2].slow_0.SVN_revision);
          muscle_driver_tmp->server_firmware_git_revision_ = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2].slow_0.SVN_server);
          muscle_driver_tmp->firmware_modified_ =
                  static_cast<bool> (static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2].slow_0.SVN_modified));

          muscle_driver_tmp->serial_number = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2 + 1].slow_1.serial_number);
          muscle_driver_tmp->assembly_date_year = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2 + 1].slow_1.assembly_date_YYYY);
          muscle_driver_tmp->assembly_date_month = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2 + 1].slow_1.assembly_date_MM);
          muscle_driver_tmp->assembly_date_day = static_cast<unsigned int> (status_data->muscle_data_packet[
                  muscle_driver_tmp->muscle_driver_id * 2 + 1].slow_1.assembly_date_DD);
          break;

        default:
          break;
      }

      // Mutual exclusion with the the initialization timeout
      boost::mutex::scoped_lock l(*lock_init_timeout_);

      // Check the message to see if everything has already been received
      if (muscle_current_state == operation_mode::device_update_state::INITIALIZATION)
      {
        if ((check_muscle_driver_data_received_flags())
            || (muscle_updater_->update_state == operation_mode::device_update_state::OPERATION))
        {
          muscle_updater_->update_state = operation_mode::device_update_state::OPERATION;
          muscle_current_state = operation_mode::device_update_state::OPERATION;
          // stop the timer
          check_init_timeout_timer.stop();

          ROS_INFO("All muscle data initialized.");
        }
      }
    }
  }

  template<class StatusType, class CommandType>
  inline void SrMuscleRobotLib<StatusType, CommandType>::set_muscle_driver_data_received_flags(unsigned int msg_type,
                                                                                               int muscle_driver_id)
  {
    if (muscle_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      map<unsigned int, unsigned int>::iterator it = from_muscle_driver_data_received_flags_.find(msg_type);
      if (it != from_muscle_driver_data_received_flags_.end())
      {
        it->second = it->second | (1 << muscle_driver_id);
      }
    }
  }

  template<class StatusType, class CommandType>
  inline bool SrMuscleRobotLib<StatusType, CommandType>::check_muscle_driver_data_received_flags()
  {
    map<unsigned int, unsigned int>::iterator it = from_muscle_driver_data_received_flags_.begin();
    for (; it != from_muscle_driver_data_received_flags_.end(); ++it)
    {
      // We want to detect when the flag for every driver is checked,
      // so, for NUM_MUSCLE_DRIVERS = 4 we want to have 00001111
      if (it->second != (1 << NUM_MUSCLE_DRIVERS) - 1)
      {
        return false;
      }
    }
    return true;
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::calibrate_joint(vector<Joint>::iterator joint_tmp,
                                                                  StatusType *status_data)
  {
    SrMuscleActuator *actuator = get_joint_actuator(joint_tmp);

    actuator->muscle_state_.raw_sensor_values_.clear();
    actuator->muscle_state_.calibrated_sensor_values_.clear();

    if (joint_tmp->joint_to_sensor.calibrate_after_combining_sensors)
    {
      // first we combine the different sensors and then we
      // calibrate the value we obtained. This is used for
      // some compound sensors ( THJ5 = cal(THJ5A + THJ5B))
      double raw_position = 0.0;
      // when combining the values, we use the coefficient imported
      // from the sensor_to_joint.yaml file (in sr_edc_launch/config)

      BOOST_FOREACH(PartialJointToSensor joint_to_sensor, joint_tmp->joint_to_sensor.joint_to_sensor_vector)
            {
              int tmp_raw = status_data->sensors[joint_to_sensor.sensor_id];
              actuator->muscle_state_.raw_sensor_values_.push_back(tmp_raw);
              raw_position += static_cast<double> (tmp_raw) * joint_to_sensor.coeff;
            }

      // and now we calibrate
      this->calibration_tmp = this->calibration_map.find(joint_tmp->joint_name);
      actuator->muscle_state_.position_unfiltered_ = this->calibration_tmp->compute(static_cast<double> (raw_position));
    }
    else
    {
      // we calibrate the different sensors first and we combine the calibrated
      // values. This is used in the joint 0s for example ( J0 = cal(J1)+cal(J2) )
      double calibrated_position = 0.0;
      PartialJointToSensor joint_to_sensor;
      string sensor_name;

      ROS_DEBUG_STREAM("Combining actuator " << joint_tmp->joint_name);

      for (unsigned int index_joint_to_sensor = 0;
           index_joint_to_sensor < joint_tmp->joint_to_sensor.joint_to_sensor_vector.size();
           ++index_joint_to_sensor)
      {
        joint_to_sensor = joint_tmp->joint_to_sensor.joint_to_sensor_vector[index_joint_to_sensor];
        sensor_name = joint_tmp->joint_to_sensor.sensor_names[index_joint_to_sensor];

        // get the raw position
        int raw_pos = status_data->sensors[joint_to_sensor.sensor_id];
        // push the new raw values
        actuator->muscle_state_.raw_sensor_values_.push_back(raw_pos);

        // calibrate and then combine
        this->calibration_tmp = this->calibration_map.find(sensor_name);
        double tmp_cal_value = this->calibration_tmp->compute(static_cast<double> (raw_pos));

        // push the new calibrated values.
        actuator->muscle_state_.calibrated_sensor_values_.push_back(tmp_cal_value);

        calibrated_position += tmp_cal_value * joint_to_sensor.coeff;

        ROS_DEBUG_STREAM("      -> " << sensor_name << " raw = " << raw_pos << " calibrated = " << calibrated_position);
      }
      actuator->muscle_state_.position_unfiltered_ = calibrated_position;
      ROS_DEBUG_STREAM("          => " << actuator->muscle_state_.position_unfiltered_);
    }
  }  // end calibrate_joint()

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::process_position_sensor_data(vector<Joint>::iterator joint_tmp,
                                                                               StatusType *status_data,
                                                                               double timestamp)
  {
    SrMuscleActuator *actuator = get_joint_actuator(joint_tmp);

    // calibrate the joint and update the position.
    calibrate_joint(joint_tmp, status_data);

    // filter the position and velocity
    pair<double, double> pos_and_velocity = joint_tmp->pos_filter.compute(actuator->muscle_state_.position_unfiltered_,
                                                                          timestamp);
    // reset the position to the filtered value
    actuator->state_.position_ = pos_and_velocity.first;
    // set the velocity to the filtered velocity
    actuator->state_.velocity_ = pos_and_velocity.second;
  }

  template<class StatusType, class CommandType>
  vector<pair<string, bool> > SrMuscleRobotLib<StatusType, CommandType>::humanize_flags(int flag)
  {
    vector<pair<string, bool> > flags;

    // 16 is the number of flags
    for (unsigned int i = 0; i < 16; ++i)
    {
      pair<string, bool> new_flag;
      // if the flag is set add the name
      if (sr_math_utils::is_bit_mask_index_true(flag, i))
      {
        if (sr_math_utils::is_bit_mask_index_true(SERIOUS_ERROR_FLAGS, i))
        {
          new_flag.second = true;
        }
        else
        {
          new_flag.second = false;
        }

        new_flag.first = error_flag_names[i];
        flags.push_back(new_flag);
      }
    }
    return flags;
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::reinitialize_motors()
  {
    // Mutual exclusion with the the initialization timeout
    boost::mutex::scoped_lock l(*lock_init_timeout_);

    // stop the timer just in case it was still running
    check_init_timeout_timer.stop();
    // Create a new MuscleUpdater object
    muscle_updater_ = shared_ptr<MuscleUpdater<CommandType> >(
            new MuscleUpdater<CommandType>(muscle_update_rate_configs_vector,
                                           operation_mode::device_update_state::INITIALIZATION));
    muscle_current_state = operation_mode::device_update_state::INITIALIZATION;
    // To reschedule the one-shot timer
    check_init_timeout_timer.setPeriod(init_max_duration);
    check_init_timeout_timer.start();
  }

  template<class StatusType, class CommandType>
  void SrMuscleRobotLib<StatusType, CommandType>::init_timer_callback(const ros::TimerEvent &event)
  {
    // Mutual exclusion with the the initialization timeout
    boost::mutex::scoped_lock l(*lock_init_timeout_);

    if (muscle_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      muscle_updater_->update_state = operation_mode::device_update_state::OPERATION;
      muscle_current_state = operation_mode::device_update_state::OPERATION;
      ROS_ERROR_STREAM(
              "Muscle Initialization Timeout: the static information in the diagnostics may not be up to date.");
    }
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class SrMuscleRobotLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}  // namespace shadow_robot

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
