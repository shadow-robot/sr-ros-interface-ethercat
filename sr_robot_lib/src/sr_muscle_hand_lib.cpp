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
#include <utility>
#include <string>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <sr_utilities/sr_math_utils.hpp>

#include "sr_robot_lib/shadow_PSTs.hpp"


using std::vector;
using std::string;
using std::pair;
using std::ostringstream;
using sr_actuator::SrMuscleActuator;
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
  const int SrMuscleHandLib<StatusType, CommandType>::nb_muscle_data = 3;

  template<class StatusType, class CommandType>
  const char *SrMuscleHandLib<StatusType, CommandType>::human_readable_muscle_data_types[nb_muscle_data] =
  {
          "muscle_data_pressure",
          "muscle_data_can_stats",
          "muscle_data_slow_misc"
  };

  template<class StatusType, class CommandType>
  const int32u SrMuscleHandLib<StatusType, CommandType>::muscle_data_types[nb_muscle_data] =
  {
    MUSCLE_DATA_PRESSURE,
    MUSCLE_DATA_CAN_STATS,
    MUSCLE_DATA_SLOW_MISC
  };

  template<class StatusType, class CommandType>
  SrMuscleHandLib<StatusType, CommandType>::SrMuscleHandLib(hardware_interface::HardwareInterface *hw,
                                                            ros::NodeHandle nh, ros::NodeHandle nhtilde,
                                                            string device_id, string joint_prefix) :
          SrMuscleRobotLib<StatusType, CommandType>(hw, nh, nhtilde, device_id, joint_prefix)
#ifdef DEBUG_PUBLISHER
          // advertise the debug service, used to set which data we want to publish on the debug topics
          , debug_service(
          nh_tilde.advertiseService("set_debug_publishers", &SrMuscleHandLib::set_debug_data_to_publish, this))
#endif
  {
    // read the muscle polling frequency from the parameter server
    this->muscle_update_rate_configs_vector = this->read_update_rate_configs("muscle_data_update_rate/", nb_muscle_data,
                                                                             human_readable_muscle_data_types,
                                                                             muscle_data_types);
    this->muscle_updater_ = shared_ptr<MuscleUpdater<CommandType> >(
            new MuscleUpdater<CommandType>(this->muscle_update_rate_configs_vector,
                                           operation_mode::device_update_state::INITIALIZATION));

    for (unsigned int i = 0; i < this->muscle_update_rate_configs_vector.size(); ++i)
    {
      // The initialization parameters (assigned a -2 in the config file)
      // are introduced in the flags map that will allow us to determine
      // if the data has been received from every muscle driver
      if (this->muscle_update_rate_configs_vector[i].when_to_update == -2)
      {
        this->from_muscle_driver_data_received_flags_[this->muscle_update_rate_configs_vector[i].what_to_update] = 0;
      }
    }

    for (unsigned int i = 0; i < NUM_MUSCLE_DRIVERS; ++i)
    {
      MuscleDriver driver(i);

      ostringstream ss;
      ss << "reset_muscle_driver_" << i;
      // initialize the reset muscle driver service
      driver.reset_driver_service =
              this->nh_tilde.template advertiseService<std_srvs::Empty::Request,
                      std_srvs::Empty::Response>(ss.str().c_str(),
                                                 boost::bind(
                                                         &SrMuscleHandLib<StatusType,
                                                                 CommandType>::reset_muscle_driver_callback,
                                                         this, _1, _2, i));

      this->muscle_drivers_vector_.push_back(driver);
    }

    vector<JointToSensor> joint_to_sensor_vect = this->read_joint_to_sensor_mapping();

    // initializing the joints vector
    vector<string> joint_names_tmp;
    vector<JointToMuscle> joint_to_muscle_map = read_joint_to_muscle_mapping();
    vector<JointToSensor> joints_to_sensors;

    ROS_ASSERT(joint_to_muscle_map.size() == JOINTS_NUM_0320);
    ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM_0320);

    for (unsigned int i = 0; i < JOINTS_NUM_0320; ++i)
    {
      joint_names_tmp.push_back(string(joint_names[i]));
      JointToSensor tmp_jts = joint_to_sensor_vect[i];
      joints_to_sensors.push_back(tmp_jts);
    }
    initialize(joint_names_tmp, joint_to_muscle_map, joint_to_sensor_vect);
  }

  template<class StatusType, class CommandType>
  void SrMuscleHandLib<StatusType, CommandType>::initialize(vector<string> joint_names,
                                                            vector<int> actuator_ids,
                                                            vector<JointToSensor> joint_to_sensors)
  {
    ROS_ERROR("This version of SrMuscleHandLib<StatusType, CommandType>::initialize should not be used");
  }

  template<class StatusType, class CommandType>
  void SrMuscleHandLib<StatusType, CommandType>::initialize(vector<string> joint_names,
                                                            vector<JointToMuscle> actuator_ids,
                                                            vector<JointToSensor> joint_to_sensors)
  {
    for (unsigned int index = 0; index < joint_names.size(); ++index)
    {
      // add the joint and the vector of joints.
      Joint joint;

      // update the joint variables
      joint.joint_name = joint_names[index];
      joint.joint_to_sensor = joint_to_sensors[index];

      if (actuator_ids[index].muscle_driver_id[0] != -1)
      {
        joint.has_actuator = true;
        shared_ptr<MuscleWrapper> muscle_wrapper(new MuscleWrapper());
        joint.actuator_wrapper = muscle_wrapper;
        muscle_wrapper->muscle_driver_id[0] = actuator_ids[index].muscle_driver_id[0];
        muscle_wrapper->muscle_driver_id[1] = actuator_ids[index].muscle_driver_id[1];
        muscle_wrapper->muscle_id[0] = actuator_ids[index].muscle_id[0];
        muscle_wrapper->muscle_id[1] = actuator_ids[index].muscle_id[1];
        muscle_wrapper->actuator = static_cast<SrMuscleActuator *> (this->hw_->getActuator(
                this->joint_prefix_ + joint.joint_name));
      }
      else
      { // no muscles associated to this joint. We only check the driver 0 assuming
        // a joint with -1 will have -1 in the driver 1 as well
        joint.has_actuator = false;
      }

      this->joints_vector.push_back(joint);
    }  // end for joints.
  }

  template<class StatusType, class CommandType>
  bool SrMuscleHandLib<StatusType, CommandType>::reset_muscle_driver_callback(std_srvs::Empty::Request &request,
                                                                              std_srvs::Empty::Response &response,
                                                                              int muscle_driver_index)
  {
    ROS_INFO_STREAM(" resetting muscle driver " << muscle_driver_index);

    this->reset_muscle_driver_queue.push(muscle_driver_index);

    return true;
  }

  template<class StatusType, class CommandType>
  vector<JointToMuscle> SrMuscleHandLib<StatusType, CommandType>::read_joint_to_muscle_mapping()
  {
    vector<JointToMuscle> muscle_map;
    string param_name = "joint_to_muscle_mapping";

    XmlRpc::XmlRpcValue mapping;
    this->nodehandle_.getParam(param_name, mapping);
    ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // iterate on all the joints
    for (int32_t i = 0; i < mapping.size(); ++i)
    {
      ROS_ASSERT(mapping[i].getType() == XmlRpc::XmlRpcValue::TypeArray);

      ROS_ASSERT(mapping[i][0].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(mapping[i][0][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(mapping[i][0][1].getType() == XmlRpc::XmlRpcValue::TypeInt);

      ROS_ASSERT(mapping[i][1].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(mapping[i][1][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(mapping[i][1][1].getType() == XmlRpc::XmlRpcValue::TypeInt);

      JointToMuscle joint_to_muscle;

      joint_to_muscle.muscle_driver_id[0] = mapping[i][0][0];
      joint_to_muscle.muscle_id[0] = mapping[i][0][1];
      joint_to_muscle.muscle_driver_id[1] = mapping[i][1][0];
      joint_to_muscle.muscle_id[1] = mapping[i][1][1];

      muscle_map.push_back(joint_to_muscle);
    }

    return muscle_map;
  }  // end read_joint_to_muscle_mapping

#ifdef DEBUG_PUBLISHER
  template <class StatusType, class CommandType>
  bool SrMuscleHandLib<StatusType,
          CommandType>::set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                                  sr_robot_msgs::SetDebugData::Response& response)
  {
    // check if the publisher_index is correct
    if (request.publisher_index < this->nb_debug_publishers_const)
    {
      if (request.motor_index > NUM_MOTORS)
      {
        response.success = false;
        return false;
      }
      if (request.motor_data_type > 0)
      {
        if ((request.motor_data_type < MOTOR_DATA_SGL) ||
            (request.motor_data_type > MOTOR_DATA_DTERM))
        {
          response.success = false;
          return false;
        }
      }
      if (!this->debug_mutex.timed_lock(boost::posix_time::microseconds(this->debug_mutex_lock_wait_time)))
      {
        response.success = false;
        return false;
      }

      this->debug_motor_indexes_and_data[request.publisher_index] = shared_ptr<pair<int, int> >(new pair<int, int>());

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

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class SrMuscleHandLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}  // namespace shadow_robot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
