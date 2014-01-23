/**
 * @file   sr_motor_hand_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Fri Jun  3 13:05:10 2011
 *
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
*
 * @brief This is a library for the etherCAT hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#include "sr_robot_lib/sr_motor_hand_lib.hpp"
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <sr_utilities/sr_math_utils.hpp>

#include "sr_robot_lib/shadow_PSTs.hpp"

namespace shadow_robot
{
  template <class StatusType, class CommandType>
  const int SrMotorHandLib<StatusType, CommandType>::nb_motor_data = 14;

  template <class StatusType, class CommandType>
  const char* SrMotorHandLib<StatusType, CommandType>::human_readable_motor_data_types[nb_motor_data] = {"sgl", "sgr", "pwm", "flags", "current",
                                                                           "voltage", "temperature", "can_num_received",
                                                                           "can_num_transmitted", "slow_data",
                                                                           "can_error_counters",
                                                                           "pterm", "iterm", "dterm"};

  template <class StatusType, class CommandType>
  const int32u SrMotorHandLib<StatusType, CommandType>::motor_data_types[nb_motor_data] = {MOTOR_DATA_SGL, MOTOR_DATA_SGR,
                                                             MOTOR_DATA_PWM, MOTOR_DATA_FLAGS,
                                                             MOTOR_DATA_CURRENT, MOTOR_DATA_VOLTAGE,
                                                             MOTOR_DATA_TEMPERATURE, MOTOR_DATA_CAN_NUM_RECEIVED,
                                                             MOTOR_DATA_CAN_NUM_TRANSMITTED, MOTOR_DATA_SLOW_MISC,
                                                             MOTOR_DATA_CAN_ERROR_COUNTERS,
                                                             MOTOR_DATA_PTERM, MOTOR_DATA_ITERM,
                                                             MOTOR_DATA_DTERM};


  template <class StatusType, class CommandType>
  SrMotorHandLib<StatusType, CommandType>::SrMotorHandLib(pr2_hardware_interface::HardwareInterface *hw) :
    SrMotorRobotLib<StatusType, CommandType>(hw)
  {
    //read the motor polling frequency from the parameter server
    this->motor_update_rate_configs_vector = this->read_update_rate_configs("motor_data_update_rate/", nb_motor_data, human_readable_motor_data_types, motor_data_types);
    this->motor_updater_ = boost::shared_ptr<generic_updater::MotorUpdater<CommandType> >(new generic_updater::MotorUpdater<CommandType>(this->motor_update_rate_configs_vector, operation_mode::device_update_state::INITIALIZATION));

    //TODO: read this from config/EEProm?
    std::vector<shadow_joints::JointToSensor > joint_to_sensor_vect = this->read_joint_to_sensor_mapping();

    //initializing the joints vector
    std::vector<std::string> joint_names_tmp;
    std::vector<int> motor_ids = read_joint_to_motor_mapping();
    std::vector<shadow_joints::JointToSensor > joints_to_sensors;
    std::vector<sr_actuator::SrGenericActuator*> actuators;

    ROS_ASSERT(motor_ids.size() == JOINTS_NUM_0220);
    ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM_0220);

    for(unsigned int i=0; i< JOINTS_NUM_0220; ++i)
    {
      joint_names_tmp.push_back(std::string(joint_names[i]));
      shadow_joints::JointToSensor tmp_jts = joint_to_sensor_vect[i];
      joints_to_sensors.push_back(tmp_jts);

      //initializing the actuators.
      sr_actuator::SrActuator* actuator = new sr_actuator::SrActuator(joint_names[i]);
      ROS_INFO_STREAM("adding actuator: "<<joint_names[i]);
      actuators.push_back( actuator );

      if(hw)
      {
        if(!hw->addActuator(actuator) )
        {
          ROS_FATAL("An actuator of the name '%s' already exists.", actuator->name_.c_str());
        }
      }
    }
    initialize(joint_names_tmp, motor_ids, joint_to_sensor_vect, actuators);

    //Initialize the motor data checker
    this->motor_data_checker = boost::shared_ptr<generic_updater::MotorDataChecker>(new generic_updater::MotorDataChecker(this->joints_vector, this->motor_updater_->initialization_configs_vector));

/*
#ifdef DEBUG_PUBLISHER
    //advertise the debug service, used to set which data we want to publish on the debug topics
    debug_service = this->nh_tilde.advertiseService( "set_debug_publishers", &SrMotorHandLib::set_debug_data_to_publish, this);
#endif
*/
  }

  template <class StatusType, class CommandType>
  SrMotorHandLib<StatusType, CommandType>::~SrMotorHandLib()
  {
    boost::ptr_vector<shadow_joints::Joint>::iterator joint = this->joints_vector.begin();
    for(;joint != this->joints_vector.end(); ++joint)
    {
      delete joint->actuator_wrapper->actuator;
    }
  }

  template <class StatusType, class CommandType>
  void SrMotorHandLib<StatusType, CommandType>::initialize(std::vector<std::string> joint_names,
                             std::vector<int> actuator_ids,
                             std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                             std::vector<sr_actuator::SrGenericActuator*> actuators)
  {
    for(unsigned int index = 0; index < joint_names.size(); ++index)
    {
      //add the joint and the vector of joints.
      this->joints_vector.push_back( new shadow_joints::Joint() );

      //get the last inserted joint
      boost::ptr_vector<shadow_joints::Joint>::reverse_iterator joint = this->joints_vector.rbegin();

      //update the joint variables
      joint->joint_name = joint_names[index];
      joint->joint_to_sensor = joint_to_sensors[index];

      if(actuator_ids[index] == -1) //no motor associated to this joint
        joint->has_actuator = false;
      else
        joint->has_actuator = true;

      boost::shared_ptr<shadow_joints::MotorWrapper> motor_wrapper ( new shadow_joints::MotorWrapper() );
      joint->actuator_wrapper    = motor_wrapper;
      motor_wrapper->motor_id = actuator_ids[index];
      motor_wrapper->actuator = actuators[index];

      std::stringstream ss;
      ss << "change_force_PID_" << joint_names[index];
      //initialize the force pid service
      //NOTE: the template keyword is needed to avoid a compiler complaint apparently due to the fact that we are using an explicit template function inside this template class
      motor_wrapper->force_pid_service = this->nh_tilde.template advertiseService<sr_robot_msgs::ForceController::Request, sr_robot_msgs::ForceController::Response>( ss.str().c_str(),
                                                                                                                                                            boost::bind( &SrMotorHandLib<StatusType, CommandType>::force_pid_callback, this, _1, _2, motor_wrapper->motor_id) );

      ss.str("");
      ss << "reset_motor_" << joint_names[index];
      //initialize the reset motor service
      motor_wrapper->reset_motor_service = this->nh_tilde.template advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>( ss.str().c_str(),
                                                                                                                                boost::bind( &SrMotorHandLib<StatusType, CommandType>::reset_motor_callback, this, _1, _2, std::pair<int,std::string>(motor_wrapper->motor_id, joint->joint_name) ) );

    } //end for joints.
  }

  template <class StatusType, class CommandType>
  bool SrMotorHandLib<StatusType, CommandType>::reset_motor_callback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response,
                                       std::pair<int,std::string> joint)
  {
    ROS_INFO_STREAM(" resetting " << joint.second << " ("<< joint.first <<")");

    this->reset_motors_queue.push(joint.first);

    //wait a few secs for the reset to be sent then resend the pids
    std::string joint_name = joint.second;
/*
    ros::Duration(5.0).sleep();
    resend_pids(joint_name, joint.first);
*/

    pid_timers[ joint_name ] = this->nh_tilde.createTimer( ros::Duration(3.0),
                                                     boost::bind(&SrMotorHandLib::resend_pids, this, joint_name, joint.first),
                                                     true );


    return true;
  }


  template <class StatusType, class CommandType>
  void SrMotorHandLib<StatusType, CommandType>::resend_pids(std::string joint_name, int motor_index)
  {
    //read the parameters from the parameter server and set the pid
    // values.
    std::stringstream full_param;

    int f, p, i, d, imax, max_pwm, sg_left, sg_right, deadband, sign;
    std::string act_name = boost::to_lower_copy(joint_name);

    full_param << act_name << "/pid/f";
    this->nodehandle_.template param<int>(full_param.str(), f, 0);
    full_param.str("");
    full_param << act_name << "/pid/p";
    this->nodehandle_.template param<int>(full_param.str(), p, 0);
    full_param.str("");
    full_param << act_name << "/pid/i";
    this->nodehandle_.template param<int>(full_param.str(), i, 0);
    full_param.str("");
    full_param << act_name << "/pid/d";
    this->nodehandle_.template param<int>(full_param.str(), d, 0);
    full_param.str("");
    full_param << act_name << "/pid/imax";
    this->nodehandle_.template param<int>(full_param.str(), imax, 0);
    full_param.str("");
    full_param << act_name << "/pid/max_pwm";
    this->nodehandle_.template param<int>(full_param.str(), max_pwm, 0);
    full_param.str("");
    full_param << act_name << "/pid/sgleftref";
    this->nodehandle_.template param<int>(full_param.str(), sg_left, 0);
    full_param.str("");
    full_param << act_name << "/pid/sgrightref";
    this->nodehandle_.template param<int>(full_param.str(), sg_right, 0);
    full_param.str("");
    full_param << act_name << "/pid/deadband";
    this->nodehandle_.template param<int>(full_param.str(), deadband, 0);
    full_param.str("");
    full_param << act_name << "/pid/sign";
    this->nodehandle_.template param<int>(full_param.str(), sign, 0);
    full_param.str("");

    sr_robot_msgs::ForceController::Request pid_request;
    pid_request.maxpwm = max_pwm;
    pid_request.sgleftref = sg_left;
    pid_request.sgrightref = sg_right;
    pid_request.f = f;
    pid_request.p = p;
    pid_request.i = i;
    pid_request.d = d;
    pid_request.imax = imax;
    pid_request.deadband = deadband;
    pid_request.sign = sign;
    sr_robot_msgs::ForceController::Response pid_response;
    bool pid_success = force_pid_callback(pid_request, pid_response, motor_index );

    //setting the backlash compensation (on or off)
    bool backlash_compensation;
    full_param << act_name << "/backlash_compensation";
    this->nodehandle_.template param<bool>(full_param.str(), backlash_compensation, true);
    full_param.str("");
    sr_robot_msgs::ChangeMotorSystemControls::Request backlash_request;
    sr_robot_msgs::MotorSystemControls motor_sys_ctrl;
    motor_sys_ctrl.motor_id = motor_index;
    motor_sys_ctrl.enable_backlash_compensation = backlash_compensation;

    if( !backlash_compensation)
      ROS_INFO_STREAM( "Setting backlash compensation to OFF for joint " << act_name );

    backlash_request.motor_system_controls.push_back(motor_sys_ctrl);
    sr_robot_msgs::ChangeMotorSystemControls::Response backlash_response;
    bool backlash_success = this->motor_system_controls_callback_( backlash_request, backlash_response );

    if( !pid_success )
      ROS_WARN_STREAM( "Didn't load the force pid settings for the motor in joint " << act_name );
    if( !backlash_success )
      ROS_WARN_STREAM( "Didn't set the backlash compensation correctly for the motor in joint " << act_name );
  }


  template <class StatusType, class CommandType>
  bool SrMotorHandLib<StatusType, CommandType>::force_pid_callback(sr_robot_msgs::ForceController::Request& request,
                                     sr_robot_msgs::ForceController::Response& response,
                                     int motor_index)
  {
    ROS_INFO_STREAM("Received new force PID parameters for motor " << motor_index);

    //Check the parameters are in the correct ranges
    if( motor_index > 20 )
    {
      ROS_WARN_STREAM(" Wrong motor index specified: " << motor_index);
      response.configured = false;;
      return false;
    }

    if( !( (request.maxpwm >= MOTOR_DEMAND_PWM_RANGE_MIN) &&
           (request.maxpwm <= MOTOR_DEMAND_PWM_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter maxpwm is out of range : " << request.maxpwm << " -> not in [" <<
                      MOTOR_DEMAND_PWM_RANGE_MIN << " ; " << MOTOR_DEMAND_PWM_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.f >= MOTOR_CONFIG_F_RANGE_MIN) &&
           (request.maxpwm <= MOTOR_CONFIG_F_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter f is out of range : " << request.f << " -> not in [" <<
                      MOTOR_CONFIG_F_RANGE_MIN << " ; " << MOTOR_CONFIG_F_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.p >= MOTOR_CONFIG_P_RANGE_MIN) &&
           (request.p <= MOTOR_CONFIG_P_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter p is out of range : " << request.p << " -> not in [" <<
                      MOTOR_CONFIG_P_RANGE_MIN << " ; " << MOTOR_CONFIG_P_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.i >= MOTOR_CONFIG_I_RANGE_MIN) &&
           (request.i <= MOTOR_CONFIG_I_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter i is out of range : " << request.i << " -> not in [" <<
                      MOTOR_CONFIG_I_RANGE_MIN << " ; " << MOTOR_CONFIG_I_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.d >= MOTOR_CONFIG_D_RANGE_MIN) &&
           (request.d <= MOTOR_CONFIG_D_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter d is out of range : " << request.d << " -> not in [" <<
                      MOTOR_CONFIG_D_RANGE_MIN << " ; " << MOTOR_CONFIG_D_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.imax >= MOTOR_CONFIG_IMAX_RANGE_MIN) &&
           (request.imax <= MOTOR_CONFIG_IMAX_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter imax is out of range : " << request.imax << " -> not in [" <<
                      MOTOR_CONFIG_IMAX_RANGE_MIN << " ; " << MOTOR_CONFIG_IMAX_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.deadband >= MOTOR_CONFIG_DEADBAND_RANGE_MIN) &&
           (request.deadband <= MOTOR_CONFIG_DEADBAND_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter deadband is out of range : " << request.deadband << " -> not in [" <<
                      MOTOR_CONFIG_DEADBAND_RANGE_MIN << " ; " << MOTOR_CONFIG_DEADBAND_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.sign >= MOTOR_CONFIG_SIGN_RANGE_MIN) &&
           (request.sign <= MOTOR_CONFIG_SIGN_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter sign is out of range : " << request.sign << " -> not in [" <<
                      MOTOR_CONFIG_SIGN_RANGE_MIN << " ; " << MOTOR_CONFIG_SIGN_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    //ok, the parameters sent are coherent, send the demand to the motor.
    this->generate_force_control_config( motor_index, request.maxpwm, request.sgleftref,
                                   request.sgrightref, request.f, request.p, request.i,
                                   request.d, request.imax, request.deadband, request.sign );

    update_force_control_in_param_server( find_joint_name(motor_index), request.maxpwm, request.sgleftref,
                                     request.sgrightref, request.f, request.p, request.i,
                                     request.d, request.imax, request.deadband, request.sign);
    response.configured = true;

    //Reinitialize motors information
    this->reinitialize_motors();

    return true;
  }

  template <class StatusType, class CommandType>
  std::string SrMotorHandLib<StatusType, CommandType>::find_joint_name(int motor_index)
  {
    for( boost::ptr_vector<shadow_joints::Joint>::iterator joint = this->joints_vector.begin();
        joint != this->joints_vector.end(); ++joint )
    {
      if( !boost::is_null(joint) ) // check for validity
      {
        if(boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint->actuator_wrapper)->motor_id == motor_index)
          return joint->joint_name;
      }
    }
    ROS_ERROR("Could not find joint name for motor index: %d", motor_index);
    return "";
  }

  template <class StatusType, class CommandType>
  void SrMotorHandLib<StatusType, CommandType>::update_force_control_in_param_server(std::string joint_name, int max_pwm, int sg_left, int sg_right, int f, int p,
                                                     int i, int d, int imax, int deadband, int sign)
  {
    if(joint_name != "")
    {
      std::stringstream full_param;
      std::string act_name = boost::to_lower_copy(joint_name);

      full_param << act_name << "/pid/f";
      this->nodehandle_.setParam(full_param.str(), f);
      full_param.str("");
      full_param << act_name << "/pid/p";
      this->nodehandle_.setParam(full_param.str(), p);
      full_param.str("");
      full_param << act_name << "/pid/i";
      this->nodehandle_.setParam(full_param.str(), i);
      full_param.str("");
      full_param << act_name << "/pid/d";
      this->nodehandle_.setParam(full_param.str(), d);
      full_param.str("");
      full_param << act_name << "/pid/imax";
      this->nodehandle_.setParam(full_param.str(), imax);
      full_param.str("");
      full_param << act_name << "/pid/max_pwm";
      this->nodehandle_.setParam(full_param.str(), max_pwm);
      full_param.str("");
      full_param << act_name << "/pid/sgleftref";
      this->nodehandle_.setParam(full_param.str(), sg_left);
      full_param.str("");
      full_param << act_name << "/pid/sgrightref";
      this->nodehandle_.setParam(full_param.str(), sg_right);
      full_param.str("");
      full_param << act_name << "/pid/deadband";
      this->nodehandle_.setParam(full_param.str(), deadband);
      full_param.str("");
      full_param << act_name << "/pid/sign";
      this->nodehandle_.setParam(full_param.str(), sign);
    }
  }

  template <class StatusType, class CommandType>
  std::vector<int> SrMotorHandLib<StatusType, CommandType>::read_joint_to_motor_mapping()
  {
    std::vector<int> motor_ids;
    std::string param_name = "joint_to_motor_mapping";

    XmlRpc::XmlRpcValue mapping;
    this->nodehandle_.getParam(param_name, mapping);
    ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t i = 0; i < mapping.size(); ++i)
    {
      ROS_ASSERT(mapping[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      motor_ids.push_back(static_cast<int>(mapping[i]));
    }

    return motor_ids;
  } //end read_joint_to_motor_mapping

/*
#ifdef DEBUG_PUBLISHER
  template <class StatusType, class CommandType>
  bool SrMotorHandLib<StatusType, CommandType>::set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                            sr_robot_msgs::SetDebugData::Response& response)
  {
    //check if the publisher_index is correct
    if( request.publisher_index < this->nb_debug_publishers_const )
    {
      if( request.motor_index > NUM_MOTORS )
      {
        response.success = false;
        return false;
      }
      if( request.motor_data_type > 0 )
      {
        if( (request.motor_data_type < MOTOR_DATA_SGL) ||
            (request.motor_data_type > MOTOR_DATA_DTERM) )
        {
          response.success = false;
          return false;
        }
      }
      if(!this->debug_mutex.timed_lock(boost::posix_time::microseconds(this->debug_mutex_lock_wait_time)))
      {
        response.success = false;
        return false;
      }

      this->debug_motor_indexes_and_data[request.publisher_index] = boost::shared_ptr<std::pair<int, int> >(new std::pair<int, int>());

      this->debug_motor_indexes_and_data[request.publisher_index]->first = request.motor_index;
      this->debug_motor_indexes_and_data[request.publisher_index]->second = request.motor_data_type;
      this->debug_mutex.unlock();
    }
    else
    {
      response.success = false;
      return false;
    }

    response.success = true;
    return true;
  }
#endif
*/
  //Only to ensure that the template class is compiled for the types we are interested in
  template class SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class SrMotorHandLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
}// end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
