/**
 * @file   sr_muscle_hand_lib.cpp
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
 * @brief This is a library for the etherCAT muscle hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#include "sr_robot_lib/sr_muscle_hand_lib.hpp"
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <sr_utilities/sr_math_utils.hpp>

#include "sr_robot_lib/shadow_PSTs.hpp"

namespace shadow_robot
{
  template <class StatusType, class CommandType>
  const int SrMuscleHandLib<StatusType, CommandType>::nb_muscle_data = 3;

  template <class StatusType, class CommandType>
  const char* SrMuscleHandLib<StatusType, CommandType>::human_readable_muscle_data_types[nb_muscle_data] = {"MUSCLE_DATA_PRESSURE",
                                                                                                            "MUSCLE_DATA_CAN_STATS",
                                                                                                            "MUSCLE_DATA_SLOW_MISC"};

  template <class StatusType, class CommandType>
  const int32u SrMuscleHandLib<StatusType, CommandType>::muscle_data_types[nb_muscle_data] = {MUSCLE_DATA_PRESSURE,
                                                                                            MUSCLE_DATA_CAN_STATS,
                                                                                            MUSCLE_DATA_SLOW_MISC};


  template <class StatusType, class CommandType>
  SrMuscleHandLib<StatusType, CommandType>::SrMuscleHandLib(pr2_hardware_interface::HardwareInterface *hw) :
    SrMotorRobotLib<StatusType, CommandType>(hw)
  {
    //read the motor polling frequency from the parameter server
    this->muscle_update_rate_configs_vector = this->read_update_rate_configs("muscle_data_update_rate/", nb_muscle_data, human_readable_muscle_data_types, muscle_data_types);
    this->muscle_updater_ = boost::shared_ptr<generic_updater::MuscleUpdater<CommandType> >(new generic_updater::MuscleUpdater<CommandType>(this->muscle_update_rate_configs_vector, operation_mode::device_update_state::INITIALIZATION));

    for(unsigned int i=0; i< this->muscle_update_rate_configs_vector.size(); ++i)
    {
      //The initialization parameters (assigned a -2 in the config file) are introduced in the flags map that will allow us to determine
      //if the data has been received from every muscle driver
      if(this->muscle_update_rate_configs_vector[i].when_to_update == -2)
        this->from_muscle_driver_data_received_flags_[this->muscle_update_rate_configs_vector[i].what_to_update] = 0;
    }

    for(unsigned int i=0; i< NUM_MUSCLE_DRIVERS; ++i)
    {
      this->muscle_drivers_vector_.push_back(new shadow_joints::MuscleDriver(i) );
    }

    std::vector<shadow_joints::JointToSensor > joint_to_sensor_vect = this->read_joint_to_sensor_mapping();

    //initializing the joints vector
    std::vector<std::string> joint_names_tmp;
    std::vector<shadow_joints::JointToMuscle> joint_to_muscle_map = read_joint_to_muscle_mapping();
    std::vector<shadow_joints::JointToSensor > joints_to_sensors;
    std::vector<sr_actuator::SrGenericActuator*> actuators;

    ROS_ASSERT(joint_to_muscle_map.size() == JOINTS_NUM_0320);
    ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM_0320);

    for(unsigned int i=0; i< JOINTS_NUM_0320; ++i)
    {
      joint_names_tmp.push_back(std::string(joint_names[i]));
      shadow_joints::JointToSensor tmp_jts = joint_to_sensor_vect[i];
      joints_to_sensors.push_back(tmp_jts);

      //initializing the actuators.
      sr_actuator::SrMuscleActuator* actuator = new sr_actuator::SrMuscleActuator(joint_names[i]);
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
    initialize(joint_names_tmp, joint_to_muscle_map, joint_to_sensor_vect, actuators);

    //Initialize the motor data checker
    this->motor_data_checker = boost::shared_ptr<generic_updater::MotorDataChecker>(new generic_updater::MotorDataChecker(this->joints_vector, this->muscle_updater_->initialization_configs_vector));


#ifdef DEBUG_PUBLISHER
    //advertise the debug service, used to set which data we want to publish on the debug topics
    debug_service = this->nh_tilde.advertiseService( "set_debug_publishers", &SrMuscleHandLib::set_debug_data_to_publish, this);
#endif
  }

  template <class StatusType, class CommandType>
  SrMuscleHandLib<StatusType, CommandType>::~SrMuscleHandLib()
  {
    boost::ptr_vector<shadow_joints::Joint>::iterator joint = this->joints_vector.begin();
    for(;joint != this->joints_vector.end(); ++joint)
    {
      delete joint->actuator_wrapper->actuator;
    }
  }

  template <class StatusType, class CommandType>
  void SrMuscleHandLib<StatusType, CommandType>::initialize(std::vector<std::string> joint_names,
                             std::vector<int> actuator_ids,
                             std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                             std::vector<sr_actuator::SrGenericActuator*> actuators)
  {

  }

  template <class StatusType, class CommandType>
  void SrMuscleHandLib<StatusType, CommandType>::initialize(std::vector<std::string> joint_names,
                             std::vector<shadow_joints::JointToMuscle> actuator_ids,
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
                                                                                                                                                            boost::bind( &SrMuscleHandLib<StatusType, CommandType>::force_pid_callback, this, _1, _2, motor_wrapper->motor_id) );

      ss.str("");
      ss << "reset_motor_" << joint_names[index];
      //initialize the reset motor service
      motor_wrapper->reset_motor_service = this->nh_tilde.template advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>( ss.str().c_str(),
                                                                                                                                boost::bind( &SrMuscleHandLib<StatusType, CommandType>::reset_motor_callback, this, _1, _2, std::pair<int,std::string>(motor_wrapper->motor_id, joint->joint_name) ) );

    } //end for joints.
  }

  template <class StatusType, class CommandType>
  bool SrMuscleHandLib<StatusType, CommandType>::reset_motor_callback(std_srvs::Empty::Request& request,
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
                                                     boost::bind(&SrMuscleHandLib::resend_pids, this, joint_name, joint.first),
                                                     true );


    return true;
  }



  template <class StatusType, class CommandType>
  std::string SrMuscleHandLib<StatusType, CommandType>::find_joint_name(int motor_index)
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
  std::vector<shadow_joints::JointToMuscle> SrMuscleHandLib<StatusType, CommandType>::read_joint_to_muscle_mapping()
  {
    std::vector<shadow_joints::JointToMuscle> muscle_map;
    std::string param_name = "joint_to_muscle_mapping";

    XmlRpc::XmlRpcValue mapping;
    this->nodehandle_.getParam(param_name, mapping);
    ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t i = 0; i < mapping.size(); ++i)
    {
      ROS_ASSERT(mapping[i].getType() == XmlRpc::XmlRpcValue::TypeArray);

      ROS_ASSERT(mapping[i][0].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(mapping[i][0][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(mapping[i][0][1].getType() == XmlRpc::XmlRpcValue::TypeInt);

      ROS_ASSERT(mapping[i][1].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(mapping[i][1][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(mapping[i][1][1].getType() == XmlRpc::XmlRpcValue::TypeInt);

      shadow_joints::JointToMuscle joint_to_muscle;

      joint_to_muscle.muscle_driver_id[0] = mapping[i][0][0];
      joint_to_muscle.muscle_id[0] = mapping[i][0][1];
      joint_to_muscle.muscle_driver_id[1] = mapping[i][1][0];
      joint_to_muscle.muscle_id[1] = mapping[i][1][1];

      muscle_map.push_back(joint_to_muscle);
    }

    return muscle_map;
  } //end read_joint_to_motor_mapping


#ifdef DEBUG_PUBLISHER
  template <class StatusType, class CommandType>
  bool SrMuscleHandLib<StatusType, CommandType>::set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                            sr_robot_msgs::SetDebugData::Response& response)
  {
    //check if the publisher_index is correct
    if( request.publisher_index < nb_debug_publishers_const )
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
      if(!debug_mutex.timed_lock(boost::posix_time::microseconds(debug_mutex_lock_wait_time)))
      {
        response.success = false;
        return false;
      }

      debug_motor_indexes_and_data[request.publisher_index] = boost::shared_ptr<std::pair<int, int> >(new std::pair<int, int>());

      debug_motor_indexes_and_data[request.publisher_index]->first = request.motor_index;
      debug_motor_indexes_and_data[request.publisher_index]->second = request.motor_data_type;
      debug_mutex.unlock();
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

  //Only to ensure that the template class is compiled for the types we are interested in
  void never_called_function()
  {
    pr2_hardware_interface::HardwareInterface *hw;
    SrMuscleHandLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND> object1 = SrMuscleHandLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>(hw);
  }
}// end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
