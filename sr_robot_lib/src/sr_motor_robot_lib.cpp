/**
 * @file   sr_motor_robot_lib.cpp
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
 * @brief This is a generic robot library for Shadow Robot's  motor-actuated Hardware.
 *
 *
 */

#include "sr_robot_lib/sr_motor_robot_lib.hpp"
#include <string>
#include <boost/foreach.hpp>

#include <sys/time.h>

#include <ros/ros.h>

#include <pr2_mechanism_msgs/ListControllers.h>

#define SERIOUS_ERROR_FLAGS PALM_0200_EDC_SERIOUS_ERROR_FLAGS
#define error_flag_names palm_0200_edc_error_flag_names

namespace shadow_robot
{

  template <class StatusType, class CommandType>
  SrMotorRobotLib<StatusType, CommandType>::SrMotorRobotLib(pr2_hardware_interface::HardwareInterface *hw)
    : SrRobotLib<StatusType, CommandType>(hw),
      motor_current_state(operation_mode::device_update_state::INITIALIZATION),
      config_index(MOTOR_CONFIG_FIRST_VALUE),
      control_type_changed_flag_(false)
  {
    lock_command_sending_ = boost::shared_ptr<boost::mutex>(new boost::mutex());

    //reading the parameters to check for a specified default control type
    // using FORCE control if no parameters are set
    control_type_.control_type = sr_robot_msgs::ControlType::FORCE;
    std::string default_control_mode;
    this->nh_tilde.template param<std::string>("default_control_mode", default_control_mode, "FORCE");
    if( default_control_mode.compare("PWM") == 0 )
    {
      ROS_INFO("Using PWM control.");
      control_type_.control_type = sr_robot_msgs::ControlType::PWM;
    }
    else
    {
      ROS_INFO("Using TORQUE control.");
    }

    //initialising the change control type service
    change_control_type_ = this->nh_tilde.advertiseService( "change_control_type", &SrMotorRobotLib::change_control_type_callback_, this);

    ///Initialising service
    motor_system_control_server_ = this->nh_tilde.advertiseService( "change_motor_system_controls", &SrMotorRobotLib::motor_system_controls_callback_, this);

#ifdef DEBUG_PUBLISHER
    this->debug_motor_indexes_and_data.resize(this->nb_debug_publishers_const);
    for( int i = 0; i < this->nb_debug_publishers_const; ++i )
    {
      std::stringstream ss;
      ss << "srh/debug_" << i;
      this->debug_publishers.push_back(this->node_handle.template advertise<std_msgs::Int16>(ss.str().c_str(),100));
    }
#endif

  }

  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::update(StatusType* status_data)
  {
    //read the PIC idle time
    this->main_pic_idle_time = status_data->idle_time_us;
    if (status_data->idle_time_us < this->main_pic_idle_time_min)
      this->main_pic_idle_time_min = status_data->idle_time_us;

    //get the current timestamp
    struct timeval tv;
    double timestamp = 0.0;
    if (gettimeofday(&tv, NULL))
    {
      ROS_WARN("SrMotorRobotLib: Failed to get system time, timestamp in state will be zero");
    }
    else
    {
      timestamp = double(tv.tv_sec) + double(tv.tv_usec) / 1.0e+6;
    }

    //First we read the joints informations
    boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = this->joints_vector.begin();
    for (; joint_tmp != this->joints_vector.end(); ++joint_tmp)
    {
      sr_actuator::SrActuatorState* actuator_state = this->get_joint_actuator_state(joint_tmp);

      boost::shared_ptr<shadow_joints::MotorWrapper> motor_wrapper = boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint_tmp->actuator_wrapper);

      motor_index_full = motor_wrapper->motor_id;
      actuator_state->is_enabled_ = 1;
      actuator_state->device_id_ = motor_index_full;
      actuator_state->halted_ = false;

      //Fill in the tactiles.
      if( this->tactiles != NULL )
        actuator_state->tactiles_ = this->tactiles->get_tactile_data();

      this->process_position_sensor_data(joint_tmp, status_data, timestamp);

      sr_actuator::SrActuator* motor_actuator = static_cast<sr_actuator::SrActuator*>(joint_tmp->actuator_wrapper->actuator);
      //filter the effort
      std::pair<double, double> effort_and_effort_d = joint_tmp->effort_filter.compute(
          motor_actuator->state_.force_unfiltered_, timestamp);
      motor_actuator->state_.last_measured_effort_ = effort_and_effort_d.first;

      //if no motor is associated to this joint, then continue
      if ((motor_index_full == -1))
        continue;

      //get the remaining information.
      bool read_motor_info = false;

      if (status_data->which_motors == 0)
      {
        //We sampled the even motor numbers
        if (motor_index_full % 2 == 0)
          read_motor_info = true;
      }
      else
      {
        //we sampled the odd motor numbers
        if (motor_index_full % 2 == 1)
          read_motor_info = true;
      }

      //the position of the motor in the message
      // is different from the motor index:
      // the motor indexes range from 0 to 19
      // while the message contains information
      // for only 10 motors.
      index_motor_in_msg = motor_index_full / 2;

      //setting the position of the motor in the message,
      // we'll print that in the diagnostics.
      motor_wrapper->msg_motor_id = index_motor_in_msg;

      //ok now we read the info and add it to the actuator state
      if (read_motor_info)
        read_additional_data(joint_tmp, status_data);
    } //end for joint

    //then we read the tactile sensors information
    this->update_tactile_info(status_data);
  } //end update()

  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::build_command(CommandType* command)
  {
    //Mutual exclusion with the change_control_type service. We have to wait until the control_type_ variable has been set.
    boost::mutex::scoped_lock l(*lock_command_sending_);

    if(control_type_changed_flag_)
    {
      if(!change_control_parameters(control_type_.control_type))
      {
        ROS_FATAL("Changing control parameters failed. Stopping real time loop to protect the robot.");
        exit(EXIT_FAILURE);
      }

      control_type_changed_flag_ = false;
    }

    if (motor_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      motor_current_state = motor_updater_->build_init_command(command);
    }
    else
    {
      //build the motor command
      motor_current_state = motor_updater_->build_command(command);
    }

    //Build the tactile sensors command
    this->build_tactile_command(command);

    ///////
    // Now we chose the command to send to the motor
    // by default we send a torque demand (we're running
    // the force control on the motors), but if we have a waiting
    // configuration, a reset command, or a motor system control
    // request then we send the configuration
    // or the reset.
    if ( reconfig_queue.empty() && reset_motors_queue.empty()
         && motor_system_control_flags_.empty() )
    {
      //no config to send
      switch( control_type_.control_type )
      {
      case sr_robot_msgs::ControlType::FORCE:
        command->to_motor_data_type = MOTOR_DEMAND_TORQUE;
        break;
      case sr_robot_msgs::ControlType::PWM:
        command->to_motor_data_type = MOTOR_DEMAND_PWM;
        break;

      default:
        command->to_motor_data_type = MOTOR_DEMAND_TORQUE;
        break;
      }

      //loop on all the joints and update their motor: we're sending commands to all the motors.
      boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = this->joints_vector.begin();
      for (; joint_tmp != this->joints_vector.end(); ++joint_tmp)
      {
        boost::shared_ptr<shadow_joints::MotorWrapper> motor_wrapper = boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint_tmp->actuator_wrapper);

        if (joint_tmp->has_actuator)
        {
          if( !this->nullify_demand_ )
          {
            //We send the computed demand
            command->motor_data[motor_wrapper->motor_id] = motor_wrapper->actuator->command_.effort_;
          }
          else
          {
            //We want to send a demand of 0
            command->motor_data[motor_wrapper->motor_id] = 0;
          }
/*
#ifdef DEBUG_PUBLISHER
          //publish the debug values for the given motors.
          // NB: debug_motor_indexes_and_data is smaller
          //     than debug_publishers.
          int publisher_index = 0;
          boost::shared_ptr<std::pair<int,int> > debug_pair;
          if( this->debug_mutex.try_lock() )
          {
            BOOST_FOREACH(debug_pair, this->debug_motor_indexes_and_data)
            {
              if( debug_pair != NULL )
              {
                shadow_joints::MotorWrapper* actuator_wrapper = static_cast<shadow_joints::MotorWrapper*>(joint_tmp->actuator_wrapper.get());
                //check if we want to publish some data for the current motor
                if( debug_pair->first == actuator_wrapper->motor_id )
                {
                  //check if it's the correct data
                  if( debug_pair->second == -1 )
                  {
                    this->msg_debug.data = joint_tmp->actuator_wrapper->actuator->command_.effort_;
                    this->debug_publishers[publisher_index].publish(this->msg_debug);
                  }
                }
              }
              publisher_index ++;
            }

            this->debug_mutex.unlock();
          } //end try_lock
#endif
*/
          joint_tmp->actuator_wrapper->actuator->state_.last_commanded_effort_ = joint_tmp->actuator_wrapper->actuator->command_.effort_;
        } //end if has_actuator
      } // end for each joint
    } //endif reconfig_queue.empty()
    else
    {
      if ( !motor_system_control_flags_.empty() )
      {
        //treat the first waiting system control and remove it from the queue
        std::vector<sr_robot_msgs::MotorSystemControls> system_controls_to_send;
        system_controls_to_send = motor_system_control_flags_.front();
        motor_system_control_flags_.pop();

        //set the correct type of command to send to the hand.
        command->to_motor_data_type = MOTOR_SYSTEM_CONTROLS;

        std::vector<sr_robot_msgs::MotorSystemControls>::iterator it;
        for( it = system_controls_to_send.begin(); it != system_controls_to_send.end(); ++it)
        {
          short combined_flags = 0;
          if( it->enable_backlash_compensation )
            combined_flags |= MOTOR_SYSTEM_CONTROL_BACKLASH_COMPENSATION_ENABLE;
          else
            combined_flags |= MOTOR_SYSTEM_CONTROL_BACKLASH_COMPENSATION_DISABLE;

          if( it->increase_sgl_tracking )
            combined_flags |= MOTOR_SYSTEM_CONTROL_SGL_TRACKING_INC;
          if( it->decrease_sgl_tracking )
            combined_flags |= MOTOR_SYSTEM_CONTROL_SGL_TRACKING_DEC;

          if( it->increase_sgr_tracking )
            combined_flags |= MOTOR_SYSTEM_CONTROL_SGR_TRACKING_INC;
          if( it->decrease_sgr_tracking )
            combined_flags |= MOTOR_SYSTEM_CONTROL_SGR_TRACKING_DEC;

          if( it->initiate_jiggling )
            combined_flags |= MOTOR_SYSTEM_CONTROL_INITIATE_JIGGLING;

          if( it->write_config_to_eeprom )
            combined_flags |= MOTOR_SYSTEM_CONTROL_EEPROM_WRITE;

          command->motor_data[ it->motor_id ] = combined_flags;
        }
      } //end if motor_system_control_flags_.empty
      else
      {
        if (!reset_motors_queue.empty())
        {
          //reset the CAN messages counters for the motor we're going to reset.
          short motor_id = reset_motors_queue.front();
          boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = this->joints_vector.begin();
          for (; joint_tmp != this->joints_vector.end(); ++joint_tmp)
          {
            boost::shared_ptr<shadow_joints::MotorWrapper> motor_wrapper = boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint_tmp->actuator_wrapper);
            sr_actuator::SrActuatorState* actuator_state = this->get_joint_actuator_state(joint_tmp);

            if( motor_wrapper->motor_id == motor_id )
            {
              actuator_state->can_msgs_transmitted_ = 0;
              actuator_state->can_msgs_received_ = 0;
            }
          }

          //we have some reset command waiting.
          // We'll send all of them
          command->to_motor_data_type = MOTOR_SYSTEM_RESET;

          while (!reset_motors_queue.empty())
          {
            motor_id = reset_motors_queue.front();
            reset_motors_queue.pop();

            // we send the MOTOR_RESET_SYSTEM_KEY
            // and the motor id (on the bus)
            crc_unions::union16 to_send;
            to_send.byte[1] = MOTOR_SYSTEM_RESET_KEY >> 8;
            if (motor_id > 9)
              to_send.byte[0] = motor_id - 10;
            else
              to_send.byte[0] = motor_id;

            command->motor_data[motor_id] = to_send.word;
          }
        } // end if reset queue not empty
        else
        {
          if (!reconfig_queue.empty())
          {
            //we have a waiting config:
            // we need to send all the config, finishing by the
            // CRC. We'll remove the config from the queue only
            // when the whole config has been sent

            // the motor data type correspond to the index
            // in the config array.
            command->to_motor_data_type = static_cast<TO_MOTOR_DATA_TYPE>(config_index);

            //convert the motor index to the index of the motor in the message
            int motor_index = reconfig_queue.front().first;

            //set the data we want to send to the given motor
            command->motor_data[motor_index] = reconfig_queue.front().second[config_index].word;

            //We're now sending the CRC. We need to send the correct CRC to
            // the motor we updated, and CRC=0 to all the other motors in its
            // group (odd/even) to tell them to ignore the new
            // configuration.
            // Once the config has been transmitted, pop the element
            // and reset the config_index to the beginning of the
            // config values
            if (config_index == static_cast<int>(MOTOR_CONFIG_CRC))
            {
              //loop on all the motors and send a CRC of 0
              // except for the motor we're reconfiguring
              for (int i = 0; i < NUM_MOTORS; ++i)
              {
                if (i != motor_index)
                  command->motor_data[i] = 0;
              }

              //reset the config_index and remove the configuration
              // we just sent from the configurations queue
              reconfig_queue.pop();
              config_index = MOTOR_CONFIG_FIRST_VALUE;
            }
            else
              ++config_index;
          } //end if reconfig queue not empty
        } // end else reset_queue.empty
      } // end else motor_system_control_flags_.empty
    } //endelse reconfig_queue.empty() && reset_queue.empty()
  }

  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                   diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    boost::ptr_vector<shadow_joints::Joint>::iterator joint = this->joints_vector.begin();
    for (; joint != this->joints_vector.end(); ++joint)
    {
      std::stringstream name;
      name.str("");
      name << "SRDMotor " << joint->joint_name;
      d.name = name.str();

      boost::shared_ptr<shadow_joints::MotorWrapper> actuator_wrapper = boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint->actuator_wrapper);

      if (joint->has_actuator)
      {
        const sr_actuator::SrActuator* sr_actuator = static_cast<sr_actuator::SrActuator*>(actuator_wrapper->actuator);
        const sr_actuator::SrMotorActuatorState* state = &(sr_actuator->state_);

        if (actuator_wrapper->actuator_ok)
        {
          if (actuator_wrapper->bad_data)
          {
            d.summary(d.WARN, "WARNING, bad CAN data received");

            d.clear();
            d.addf("Motor ID", "%d", actuator_wrapper->motor_id);
          }
          else //the data is good
          {
            d.summary(d.OK, "OK");

            d.clear();
            d.addf("Motor ID", "%d", actuator_wrapper->motor_id);
            d.addf("Motor ID in message", "%d", actuator_wrapper->msg_motor_id);
            d.addf("Serial Number", "%d", state->serial_number );
            d.addf("Assembly date", "%d / %d / %d", state->assembly_data_day, state->assembly_data_month, state->assembly_data_year );

            d.addf("Strain Gauge Left", "%d", state->strain_gauge_left_);
            d.addf("Strain Gauge Right", "%d", state->strain_gauge_right_);

            //if some flags are set
            std::stringstream ss;
            if (state->flags_.size() > 0)
            {
              int flags_seriousness = d.OK;
              std::pair<std::string, bool> flag;
              BOOST_FOREACH(flag, state->flags_)
              {
                //Serious error flag
                if (flag.second)
                  flags_seriousness = d.ERROR;

                if (flags_seriousness != d.ERROR)
                  flags_seriousness = d.WARN;
                ss << flag.first << " | ";
              }
              d.summary(flags_seriousness, ss.str().c_str());
            }
            else
              ss << " None";
            d.addf("Motor Flags", "%s", ss.str().c_str());

            d.addf("Measured PWM", "%d", state->pwm_);
            d.addf("Measured Current", "%f", state->last_measured_current_);
            d.addf("Measured Voltage", "%f", state->motor_voltage_);
            d.addf("Measured Effort", "%f", state->last_measured_effort_);
            d.addf("Temperature", "%f", state->temperature_);

            d.addf("Unfiltered position", "%f", state->position_unfiltered_);
            d.addf("Unfiltered force", "%f", state->force_unfiltered_);

            d.addf("Gear Ratio", "%d", state->motor_gear_ratio);

            d.addf("Number of CAN messages received", "%lld", state->can_msgs_received_);
            d.addf("Number of CAN messages transmitted", "%lld", state->can_msgs_transmitted_);

            d.addf("Force control Pterm", "%d", state->force_control_pterm);
            d.addf("Force control Iterm", "%d", state->force_control_iterm);
            d.addf("Force control Dterm", "%d", state->force_control_dterm);

            d.addf("Force control F", "%d", state->force_control_f_);
            d.addf("Force control P", "%d", state->force_control_p_);
            d.addf("Force control I", "%d", state->force_control_i_);
            d.addf("Force control D", "%d", state->force_control_d_);
            d.addf("Force control Imax", "%d", state->force_control_imax_);
            d.addf("Force control Deadband", "%d", state->force_control_deadband_);
            d.addf("Force control Frequency", "%d", state->force_control_frequency_);

            if (state->force_control_sign_ == 0)
              d.addf("Force control Sign", "+");
            else
              d.addf("Force control Sign", "-");

            d.addf("Last Commanded Effort", "%f", state->last_commanded_effort_);

            d.addf("Encoder Position", "%f", state->position_);

            if (state->firmware_modified_)
              d.addf("Firmware svn revision (server / pic / modified)", "%d / %d / True",
                     state->server_firmware_svn_revision_, state->pic_firmware_svn_revision_);
            else
              d.addf("Firmware svn revision (server / pic / modified)", "%d / %d / False",
                     state->server_firmware_svn_revision_, state->pic_firmware_svn_revision_);
          }
        }
        else
        {
          d.summary(d.ERROR, "Motor error");
          d.clear();
          d.addf("Motor ID", "%d", actuator_wrapper->motor_id);
        }
      }
      else
      {
        d.summary(d.OK, "No motor associated to this joint");
        d.clear();
      }
      vec.push_back(d);

    } //end for each joints

  }


  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::read_additional_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data)
  {
    //check the masks to see if the CAN messages arrived to the motors
    //the flag should be set to 1 for each motor
    joint_tmp->actuator_wrapper->actuator_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_arrived,
                                                                       motor_index_full);

    //check the masks to see if a bad CAN message arrived
    //the flag should be 0
    joint_tmp->actuator_wrapper->bad_data = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_had_errors,
                                                                       index_motor_in_msg);

    crc_unions::union16 tmp_value;

    if (joint_tmp->actuator_wrapper->actuator_ok && !(joint_tmp->actuator_wrapper->bad_data))
    {
      sr_actuator::SrActuator* actuator = static_cast<sr_actuator::SrActuator*>(joint_tmp->actuator_wrapper->actuator);
      shadow_joints::MotorWrapper* actuator_wrapper = static_cast<shadow_joints::MotorWrapper*>(joint_tmp->actuator_wrapper.get());
/*
#ifdef DEBUG_PUBLISHER
      int publisher_index = 0;
      //publish the debug values for the given motors.
      // NB: debug_motor_indexes_and_data is smaller
      //     than debug_publishers.
      boost::shared_ptr<std::pair<int,int> > debug_pair;

      if( this->debug_mutex.try_lock() )
      {
        BOOST_FOREACH(debug_pair, this->debug_motor_indexes_and_data)
        {
          if( debug_pair != NULL )
          {
            shadow_joints::MotorWrapper* actuator_wrapper = static_cast<shadow_joints::MotorWrapper*>(joint_tmp->actuator_wrapper.get());
            
            //check if we want to publish some data for the current motor
            if( debug_pair->first == actuator_wrapper->motor_id )
            {
              //if < 0, then we're not asking for a FROM_MOTOR_DATA_TYPE
              if( debug_pair->second > 0 )
              {
                //check if it's the correct data
                if( debug_pair->second == status_data->motor_data_type )
                {
                  this->msg_debug.data = status_data->motor_data_packet[index_motor_in_msg].misc;
                  this->debug_publishers[publisher_index].publish(this->msg_debug);
                }
              }
            }
          }
          publisher_index ++;
        }

        this->debug_mutex.unlock();
      } //end try_lock
#endif
*/
      //we received the data and it was correct
      bool read_torque = true;
      switch (status_data->motor_data_type)
      {
      case MOTOR_DATA_SGL:
        actuator->state_.strain_gauge_left_ =
          static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].misc);

#ifdef DEBUG_PUBLISHER
        if( actuator_wrapper->motor_id == 19 )
        {
          //ROS_ERROR_STREAM("SGL " <<actuator->state_.strain_gauge_left_);
          this->msg_debug.data = actuator->state_.strain_gauge_left_;
          this->debug_publishers[0].publish(this->msg_debug);
        }
#endif
        break;
      case MOTOR_DATA_SGR:
        actuator->state_.strain_gauge_right_ =
          static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].misc);

#ifdef DEBUG_PUBLISHER
        if( actuator_wrapper->motor_id == 19 )
        {
          //ROS_ERROR_STREAM("SGR " <<actuator->state_.strain_gauge_right_);
          this->msg_debug.data = actuator->state_.strain_gauge_right_;
          this->debug_publishers[1].publish(this->msg_debug);
        }
#endif
        break;
      case MOTOR_DATA_PWM:
        actuator->state_.pwm_ =
          static_cast<int>(static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].misc));

#ifdef DEBUG_PUBLISHER
        if( actuator_wrapper->motor_id == 19 )
        {
          //ROS_ERROR_STREAM("SGR " <<actuator->state_.strain_gauge_right_);
          this->msg_debug.data = actuator->state_.pwm_;
          this->debug_publishers[2].publish(this->msg_debug);
        }
#endif
        break;
      case MOTOR_DATA_FLAGS:
        actuator->state_.flags_ = humanize_flags(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_CURRENT:
        //we're receiving the current in milli amps
        actuator->state_.last_measured_current_ =
          static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc))
          / 1000.0;

#ifdef DEBUG_PUBLISHER
        if( actuator_wrapper->motor_id == 19 )
        {
          //ROS_ERROR_STREAM("Current " <<actuator->state_.last_measured_current_);
          this->msg_debug.data = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
          this->debug_publishers[3].publish(this->msg_debug);
        }
#endif
        break;
      case MOTOR_DATA_VOLTAGE:
        actuator->state_.motor_voltage_ =
          static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)) / 256.0;

#ifdef DEBUG_PUBLISHER
        if( actuator_wrapper->motor_id == 19 )
        {
          //ROS_ERROR_STREAM("Voltage " <<actuator->state_.motor_voltage_);
          this->msg_debug.data = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
          this->debug_publishers[4].publish(this->msg_debug);
        }
#endif
        break;
      case MOTOR_DATA_TEMPERATURE:
        actuator->state_.temperature_ =
          static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)) / 256.0;
        break;
      case MOTOR_DATA_CAN_NUM_RECEIVED:
        // those are 16 bits values and will overflow -> we compute the real value.
        // This needs to be updated faster than the overflowing period (which should be roughly every 30s)
        actuator->state_.can_msgs_received_ = sr_math_utils::counter_with_overflow(
          actuator->state_.can_msgs_received_,
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
        break;
      case MOTOR_DATA_CAN_NUM_TRANSMITTED:
        // those are 16 bits values and will overflow -> we compute the real value.
        // This needs to be updated faster than the overflowing period (which should be roughly every 30s)
        actuator->state_.can_msgs_transmitted_ = sr_math_utils::counter_with_overflow(
          actuator->state_.can_msgs_transmitted_,
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
        break;

      case MOTOR_DATA_SLOW_MISC:
        //We received a slow data:
        // the slow data type is contained in .torque, while
        // the actual data is in .misc.
        // so we won't read torque information from .torque
        read_torque = false;

        switch (static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].torque))
        {
        case MOTOR_SLOW_DATA_SVN_REVISION:
          actuator->state_.pic_firmware_svn_revision_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_SVN_SERVER_REVISION:
          actuator->state_.server_firmware_svn_revision_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_SVN_MODIFIED:
          actuator->state_.firmware_modified_ =
            static_cast<bool>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_SERIAL_NUMBER_LOW:
          actuator->state_.set_serial_number_low (
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)) );
          break;
        case MOTOR_SLOW_DATA_SERIAL_NUMBER_HIGH:
          actuator->state_.set_serial_number_high (
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)) );
          break;
        case MOTOR_SLOW_DATA_GEAR_RATIO:
          actuator->state_.motor_gear_ratio =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_ASSEMBLY_DATE_YYYY:
          actuator->state_.assembly_data_year =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_ASSEMBLY_DATE_MMDD:
          actuator->state_.assembly_data_month =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)
                                      >> 8);
          actuator->state_.assembly_data_day =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)
                                      && 0x00FF);
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_F:
          actuator->state_.force_control_f_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_P:
          actuator->state_.force_control_p_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_I:
          actuator->state_.force_control_i_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_D:
          actuator->state_.force_control_d_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_IMAX:
          actuator->state_.force_control_imax_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_DEADSIGN:
          tmp_value.word = status_data->motor_data_packet[index_motor_in_msg].misc;
          actuator->state_.force_control_deadband_ = static_cast<int>(tmp_value.byte[0]);
          actuator->state_.force_control_sign_ = static_cast<int>(tmp_value.byte[1]);
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_FREQUENCY:
          actuator->state_.force_control_frequency_ =
            static_cast<unsigned int>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
          break;

        default:
          break;
        }
        break;

      case MOTOR_DATA_CAN_ERROR_COUNTERS:
        actuator->state_.can_error_counters =
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_PTERM:
        actuator->state_.force_control_pterm =
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_ITERM:
        actuator->state_.force_control_iterm =
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_DTERM:
        actuator->state_.force_control_dterm =
          static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;

      default:
        break;
      }

      if (read_torque)
      {
        actuator->state_.force_unfiltered_ =
          static_cast<double>(static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].torque));

#ifdef DEBUG_PUBLISHER
	if( actuator_wrapper->motor_id == 19 )
        {
          this->msg_debug.data = static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].torque);
          this->debug_publishers[5].publish(this->msg_debug);
        }
#endif
      }

      //Check the message to see if everything has already been received
      if (motor_current_state == operation_mode::device_update_state::INITIALIZATION)
      {
        if (motor_data_checker->check_message(
              joint_tmp, status_data->motor_data_type,
              static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].torque)))
        {
          motor_updater_->update_state = operation_mode::device_update_state::OPERATION;
          motor_current_state = operation_mode::device_update_state::OPERATION;

          ROS_INFO("All motors were initialized.");
        }
      }
    }
  }

  template <class StatusType, class CommandType>
  std::vector<std::pair<std::string, bool> > SrMotorRobotLib<StatusType, CommandType>::humanize_flags(int flag)
  {
    std::vector<std::pair<std::string, bool> > flags;

    //16 is the number of flags
    for (unsigned int i = 0; i < 16; ++i)
    {
      std::pair<std::string, bool> new_flag;
      //if the flag is set add the name
      if (sr_math_utils::is_bit_mask_index_true(flag, i))
      {
        if (sr_math_utils::is_bit_mask_index_true(SERIOUS_ERROR_FLAGS, i))
          new_flag.second = true;
        else
          new_flag.second = false;

        new_flag.first = error_flag_names[i];
        flags.push_back(new_flag);
      }
    }
    return flags;
  }

  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::generate_force_control_config(int motor_index, int max_pwm, int sg_left, int sg_right, int f, int p,
                                                 int i, int d, int imax, int deadband, int sign)
  {
    ROS_INFO_STREAM(
      "Setting new pid values for motor" << motor_index << ": max_pwm="<< max_pwm <<" sgleftref=" << sg_left << " sgrightref=" << sg_right << " f=" << f << " p=" << p << " i=" << i << " d="<< d << " imax=" << imax << " deadband="<< deadband << " sign=" << sign);

    //the vector is of the size of the TO_MOTOR_DATA_TYPE enum.
    //the value of the element at a given index is the value
    //for the given MOTOR_CONFIG.
    std::vector<crc_unions::union16> full_config(MOTOR_CONFIG_CRC + 1);
    crc_unions::union16 value;

    value.word = max_pwm;
    full_config.at(MOTOR_CONFIG_MAX_PWM) = value;

    value.byte[0] = sg_left;
    value.byte[1] = sg_right;
    full_config.at(MOTOR_CONFIG_SG_REFS) = value;

    value.word = f;
    full_config.at(MOTOR_CONFIG_F) = value;

    value.word = p;
    full_config.at(MOTOR_CONFIG_P) = value;

    value.word = i;
    full_config.at(MOTOR_CONFIG_I) = value;

    value.word = d;
    full_config.at(MOTOR_CONFIG_D) = value;

    value.word = imax;
    full_config.at(MOTOR_CONFIG_IMAX) = value;

    value.byte[0] = deadband;
    value.byte[1] = sign;
    full_config.at(MOTOR_CONFIG_DEADBAND_SIGN) = value;
    ROS_DEBUG_STREAM(
      "deadband: " << static_cast<int>(static_cast<int8u>(value.byte[0]) ) << " value: " << static_cast<int16u>(value.word));

    //compute crc
    crc_result = 0;
    for (unsigned int i = MOTOR_CONFIG_FIRST_VALUE; i <= MOTOR_CONFIG_LAST_VALUE; ++i)
    {
      crc_byte = full_config.at(i).byte[0];
      INSERT_CRC_CALCULATION_HERE;

      crc_byte = full_config.at(i).byte[1];
      INSERT_CRC_CALCULATION_HERE;
    }

    //never send a CRC of 0, send 1 if the
    // computed CRC is 0 (0 is a code for
    // ignoring the config)
    if (crc_result == 0)
      crc_result = 1;
    value.word = crc_result;
    full_config.at(MOTOR_CONFIG_CRC) = value;

    ForceConfig config;
    config.first = motor_index;
    config.second = full_config;
    //push the new config to the configuration queue
    reconfig_queue.push(config);
  }

  template <class StatusType, class CommandType>
  void SrMotorRobotLib<StatusType, CommandType>::reinitialize_motors()
  {
    //Create a new MotorUpdater object
    motor_updater_ = boost::shared_ptr<generic_updater::MotorUpdater<CommandType> >(new generic_updater::MotorUpdater<CommandType>(motor_update_rate_configs_vector, operation_mode::device_update_state::INITIALIZATION));
    motor_current_state = operation_mode::device_update_state::INITIALIZATION;
    //Initialize the motor data checker
    motor_data_checker = boost::shared_ptr<generic_updater::MotorDataChecker>(new generic_updater::MotorDataChecker(this->joints_vector, motor_updater_->initialization_configs_vector));
  }

  template <class StatusType, class CommandType>
  bool SrMotorRobotLib<StatusType, CommandType>::change_control_type_callback_( sr_robot_msgs::ChangeControlType::Request& request,
                                                  sr_robot_msgs::ChangeControlType::Response& response )
  {
    //querying which we're control type we're using currently.
    if( request.control_type.control_type == sr_robot_msgs::ControlType::QUERY )
    {
      response.result = control_type_;
      return true;
    }

    //We're not querying the control type
    if( (request.control_type.control_type != sr_robot_msgs::ControlType::PWM) &&
        (request.control_type.control_type != sr_robot_msgs::ControlType::FORCE) )
    {
      std::string ctrl_type_text = "";
      if(control_type_.control_type == sr_robot_msgs::ControlType::FORCE)
        ctrl_type_text = "FORCE";
      else
        ctrl_type_text = "PWM";

      ROS_ERROR_STREAM(" The value you specified for the control type (" << request.control_type
                       << ") is incorrect. Using " << ctrl_type_text << " control.");

      response.result = control_type_;
      return false;
    }

    if(control_type_.control_type != request.control_type.control_type)
    {
      //Mutual exclusion with the build_command() function. We have to wait until the current motor command has been built.
      boost::mutex::scoped_lock l(*lock_command_sending_);

      ROS_WARN("Changing control type");

      control_type_ = request.control_type;
      //Flag to signal that there has been a change in the value of control_type_ and certain actions are required.
      //The flag is set in the callback function of the change_control_type_ service.
      //The flag is checked in build_command() and the necessary actions are taken there.
      //These actions involve calling services in the controller manager and all the active controllers. This is the
      //reason why we don't do it directly in the callback function. As we use a single thread to serve the callbacks,
      //doing so would cause a deadlock, thus we do it in the realtime loop thread instead.
      control_type_changed_flag_ = true;
    }

    response.result = control_type_;
    return true;
  }

  template <class StatusType, class CommandType>
  bool SrMotorRobotLib<StatusType, CommandType>::change_control_parameters(int16_t control_type)
  {
    bool success = true;
    std::string env_variable;
    std::string param_value;
    ros::NodeHandle nh;

    if( control_type == sr_robot_msgs::ControlType::PWM)
    {
      env_variable = "PWM_CONTROL=1";
      param_value = "PWM";
    }
    else
    {
      env_variable = "PWM_CONTROL=0";
      param_value = "FORCE";
    }

    // Read the namespace of the node.
    // The ns will be passed as an argument to the sr_edc_default_controllers.launch
    // so that the parameters in it will be loaded inside the correct namespace
    std::string ns = ros::this_node::getNamespace();
    std::string arguments = "";

    if(ns.compare("/") == 0)
    {
      ROS_DEBUG("Using base namespace: %s", ns.c_str());
    }
    else if (ns.find("//") == 0)
    {
      ns.erase(0,2);
      ROS_DEBUG("Node namespace: %s", ns.c_str());
      arguments += " set_namespace:=1 namespace:=" + ns;
    }
    else
    {
      ROS_ERROR("Node namespace: %s", ns.c_str());
    }
    arguments = " set_namespace:=0";
    // Read the config_dir prefix from the parameter server
    // The config_dir will be passed as an argument to the sr_edc_default_controllers.launch
    // so that the parameters in it will be read from the correct files
    std::string config_dir = "";
    nh.template param<std::string>("config_dir", config_dir, "");
    ROS_DEBUG("config_dir: %s", config_dir.c_str());
    arguments += " config_dir:=" + config_dir;
    ROS_INFO("arguments: %s", arguments.c_str());

    int result = system((env_variable + " roslaunch sr_ethercat_hand_config sr_edc_default_controllers.launch" + arguments).c_str());

    if(result == 0)
    {
      ROS_WARN("New parameters loaded successfully on Parameter Server");

      this->nh_tilde.setParam("default_control_mode", param_value);

      ros::ServiceClient list_ctrl_client = nh.template serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");
      pr2_mechanism_msgs::ListControllers controllers_list;

      if (list_ctrl_client.call(controllers_list))
      {
        for(unsigned int i=0; i < controllers_list.response.controllers.size(); ++i)
        {
          ros::ServiceClient reset_gains_client = nh.template serviceClient<std_srvs::Empty>(controllers_list.response.controllers.at(i) + "/reset_gains");
          std_srvs::Empty empty_message;
          if (!reset_gains_client.call(empty_message))
          {
            ROS_ERROR_STREAM("Failed to reset gains for controller: " << controllers_list.response.controllers.at(i));
            return false;
          }
        }
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    return success;
  }

  template <class StatusType, class CommandType>
  bool SrMotorRobotLib<StatusType, CommandType>::motor_system_controls_callback_( sr_robot_msgs::ChangeMotorSystemControls::Request& request,
                                                    sr_robot_msgs::ChangeMotorSystemControls::Response& response )
  {
    std::vector<sr_robot_msgs::MotorSystemControls> tmp_motor_controls;

    response.result = sr_robot_msgs::ChangeMotorSystemControls::Response::SUCCESS;
    bool no_motor_id_out_of_range = true;

    for( unsigned int i=0; i < request.motor_system_controls.size(); ++i)
    {
      if( request.motor_system_controls[i].motor_id >= NUM_MOTORS ||
          request.motor_system_controls[i].motor_id < 0)
      {
        response.result = sr_robot_msgs::ChangeMotorSystemControls::Response::MOTOR_ID_OUT_OF_RANGE;
        no_motor_id_out_of_range = false;
      }
      else
      {
        //only pushes the demands with a correct motor_id
        tmp_motor_controls.push_back( request.motor_system_controls[i] );
      }
    }

    //add the request to the queue if it's not empty
    if( tmp_motor_controls.size() > 0 )
      motor_system_control_flags_.push( tmp_motor_controls );

    return no_motor_id_out_of_range;
  }

  //Only to ensure that the template class is compiled for the types we are interested in
  template class SrMotorRobotLib<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class SrMotorRobotLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
} //end namespace

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
