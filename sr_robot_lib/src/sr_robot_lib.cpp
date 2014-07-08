/**
 * @file   sr_robot_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jun 22 10:06:14 2011
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
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 *
 *
 */

#include "sr_robot_lib/sr_robot_lib.hpp"
#include <string>
#include <boost/foreach.hpp>

#include <sys/time.h>

#include <ros/ros.h>

#include "sr_robot_lib/shadow_PSTs.hpp"
#include "sr_robot_lib/biotac.hpp"
#include "sr_robot_lib/UBI0.hpp"
#include <controller_manager_msgs/ListControllers.h>

#define SERIOUS_ERROR_FLAGS PALM_0200_EDC_SERIOUS_ERROR_FLAGS
#define error_flag_names palm_0200_edc_error_flag_names

namespace shadow_robot
{
#ifdef DEBUG_PUBLISHER
//max of 20 publishers for debug
  template <class StatusType, class CommandType>
  const int SrRobotLib<StatusType, CommandType>::nb_debug_publishers_const = 20;
  //template <class StatusType, class CommandType>
  //const int SrRobotLib<StatusType, CommandType>::debug_mutex_lock_wait_time = 100;
#endif

  template <class StatusType, class CommandType>
  const int SrRobotLib<StatusType, CommandType>::nb_sensor_data = 32;

  template <class StatusType, class CommandType>
  const char* SrRobotLib<StatusType, CommandType>::human_readable_sensor_data_types[nb_sensor_data] = {"TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ",
                                                                             "TACTILE_SENSOR_TYPE_MANUFACTURER",
                                                                             "TACTILE_SENSOR_TYPE_SERIAL_NUMBER",
                                                                             "TACTILE_SENSOR_TYPE_SOFTWARE_VERSION",
                                                                             "TACTILE_SENSOR_TYPE_PCB_VERSION",
                                                                             "TACTILE_SENSOR_TYPE_WHICH_SENSORS",
                                                                             "TACTILE_SENSOR_TYPE_PST3_PRESSURE_TEMPERATURE",
                                                                             "TACTILE_SENSOR_TYPE_PST3_PRESSURE_RAW_ZERO_TRACKING",
                                                                             "TACTILE_SENSOR_TYPE_PST3_DAC_VALUE",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_1",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_2",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_3",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_4",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_5",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_6",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_7",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_8",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_9",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_10",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_11",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_12",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_13",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_14",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_15",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_16",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_17",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_18",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_19",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_PDC",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_TAC",
                                                                             "TACTILE_SENSOR_TYPE_BIOTAC_TDC",
                                                                             "TACTILE_SENSOR_TYPE_UBI0_TACTILE"
                                                                             };

  template <class StatusType, class CommandType>
  const int32u SrRobotLib<StatusType, CommandType>::sensor_data_types[nb_sensor_data] = {TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ,
                                                               TACTILE_SENSOR_TYPE_MANUFACTURER,
                                                               TACTILE_SENSOR_TYPE_SERIAL_NUMBER,
                                                               TACTILE_SENSOR_TYPE_SOFTWARE_VERSION,
                                                               TACTILE_SENSOR_TYPE_PCB_VERSION,
                                                               TACTILE_SENSOR_TYPE_WHICH_SENSORS,
                                                               TACTILE_SENSOR_TYPE_PST3_PRESSURE_TEMPERATURE,
                                                               TACTILE_SENSOR_TYPE_PST3_PRESSURE_RAW_ZERO_TRACKING,
                                                               TACTILE_SENSOR_TYPE_PST3_DAC_VALUE,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_1,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_2,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_3,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_4,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_5,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_6,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_7,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_8,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_9,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_10,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_11,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_12,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_13,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_14,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_15,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_16,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_17,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_18,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_19,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_PDC,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_TAC,
                                                               TACTILE_SENSOR_TYPE_BIOTAC_TDC,
                                                               TACTILE_SENSOR_TYPE_UBI0_TACTILE
  };

  template <class StatusType, class CommandType>
  SrRobotLib<StatusType, CommandType>::SrRobotLib(hardware_interface::HardwareInterface *hw)
    : main_pic_idle_time(0), main_pic_idle_time_min(1000), nullify_demand_(false),
      tactile_current_state(operation_mode::device_update_state::INITIALIZATION),
      hw_(static_cast<ros_ethercat_model::RobotState*>(hw)),
      nh_tilde("~"),

      //advertise the service to nullify the demand sent to the motor
      // this makes it possible to easily stop the controllers.
      nullify_demand_server_(nh_tilde.advertiseService("nullify_demand", &SrRobotLib::nullify_demand_callback, this))
  {
    //read the generic sensor polling frequency from the parameter server
    this->generic_sensor_update_rate_configs_vector = this->read_update_rate_configs("generic_sensor_data_update_rate/", nb_sensor_data, human_readable_sensor_data_types, sensor_data_types);
    this->tactiles_init = boost::shared_ptr<tactiles::GenericTactiles<StatusType, CommandType> >( new tactiles::GenericTactiles<StatusType, CommandType>(this->generic_sensor_update_rate_configs_vector, operation_mode::device_update_state::INITIALIZATION) );

    //read the pst3 sensor polling frequency from the parameter server
    this->pst3_sensor_update_rate_configs_vector = this->read_update_rate_configs("pst3_sensor_data_update_rate/", nb_sensor_data, human_readable_sensor_data_types, sensor_data_types);
    //read the biotac sensor polling frequency from the parameter server
    this->biotac_sensor_update_rate_configs_vector = this->read_update_rate_configs("biotac_sensor_data_update_rate/", nb_sensor_data, human_readable_sensor_data_types, sensor_data_types);
    //read the UBI0 sensor polling frequency from the parameter server
    this->ubi0_sensor_update_rate_configs_vector = this->read_update_rate_configs("ubi0_sensor_data_update_rate/", nb_sensor_data, human_readable_sensor_data_types, sensor_data_types);

    //initialize the calibration map
    this->calibration_map = this->read_joint_calibration();

    //initialises self tests (false as this is not a simulated hand\)
    self_tests_.reset( new SrSelfTest(false) );
    self_test_thread_.reset(new boost::thread(boost::bind(&SrRobotLib::checkSelfTests, this)));
  }

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::calibrate_joint(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data)
  {
    sr_actuator::SrActuatorState* actuator_state = get_joint_actuator_state(joint_tmp);

    actuator_state->raw_sensor_values_.clear();
    actuator_state->calibrated_sensor_values_.clear();

    if (joint_tmp->joint_to_sensor.calibrate_after_combining_sensors)
    {
      //first we combine the different sensors and then we
      // calibrate the value we obtained. This is used for
      // some compound sensors ( THJ5 = cal(THJ5A + THJ5B))
      double raw_position = 0.0;
      //when combining the values, we use the coefficient imported
      // from the sensor_to_joint.yaml file (in sr_edc_launch/config)
      BOOST_FOREACH(shadow_joints::PartialJointToSensor joint_to_sensor, joint_tmp->joint_to_sensor.joint_to_sensor_vector)
      {
        int tmp_raw = status_data->sensors[joint_to_sensor.sensor_id];
        actuator_state->raw_sensor_values_.push_back(tmp_raw);
        raw_position += static_cast<double>(tmp_raw) * joint_to_sensor.coeff;
      }

      //and now we calibrate
      calibration_tmp = calibration_map.find(joint_tmp->joint_name);
      actuator_state->position_unfiltered_ = calibration_tmp->compute(static_cast<double>(raw_position));
    }
    else
    {
      //we calibrate the different sensors first and we combine the calibrated
      //values. This is used in the joint 0s for example ( J0 = cal(J1)+cal(J2) )
      double calibrated_position = 0.0;
      shadow_joints::PartialJointToSensor joint_to_sensor;
      std::string sensor_name;

      ROS_DEBUG_STREAM("Combining actuator " << joint_tmp->joint_name);

      for (unsigned int index_joint_to_sensor = 0;
           index_joint_to_sensor < joint_tmp->joint_to_sensor.joint_to_sensor_vector.size(); ++index_joint_to_sensor)
      {
        joint_to_sensor = joint_tmp->joint_to_sensor.joint_to_sensor_vector[index_joint_to_sensor];
        sensor_name = joint_tmp->joint_to_sensor.sensor_names[index_joint_to_sensor];

        //get the raw position
        int raw_pos = status_data->sensors[joint_to_sensor.sensor_id];
        //push the new raw values
        actuator_state->raw_sensor_values_.push_back(raw_pos);

        //calibrate and then combine
        calibration_tmp = calibration_map.find(sensor_name);
        double tmp_cal_value = calibration_tmp->compute(static_cast<double>(raw_pos));

        //push the new calibrated values.
        actuator_state->calibrated_sensor_values_.push_back(tmp_cal_value);

        calibrated_position += tmp_cal_value * joint_to_sensor.coeff;

        ROS_DEBUG_STREAM("      -> "<< sensor_name<< " raw = " << raw_pos << " calibrated = " << calibrated_position);
      }
      actuator_state->position_unfiltered_ = calibrated_position;
      ROS_DEBUG_STREAM("          => "<< actuator_state->position_unfiltered_);
    }
  } //end calibrate_joint()

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::reinitialize_sensors()
  {
    //Create a new GenericTactiles object
    tactiles_init = boost::shared_ptr<tactiles::GenericTactiles<StatusType, CommandType> >( new tactiles::GenericTactiles<StatusType, CommandType>(generic_sensor_update_rate_configs_vector, operation_mode::device_update_state::INITIALIZATION) );
    tactile_current_state = operation_mode::device_update_state::INITIALIZATION;
  }

  template <class StatusType, class CommandType>
  bool SrRobotLib<StatusType, CommandType>::nullify_demand_callback( sr_robot_msgs::NullifyDemand::Request& request,
                                            sr_robot_msgs::NullifyDemand::Response& response )
  {
    if( request.nullify_demand )
      ROS_INFO_STREAM("Nullifying the demand sent to the motor. Will ignore the values computed by the controllers and send 0.");
    else
      ROS_INFO_STREAM("Using the value computed by the controllers to send the demands to the motors.");

    nullify_demand_ = request.nullify_demand;
    return true;
  }

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::process_position_sensor_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp, StatusType* status_data, double timestamp)
  {
    sr_actuator::SrActuatorState* actuator_state = get_joint_actuator_state(joint_tmp);

    //calibrate the joint and update the position.
    calibrate_joint(joint_tmp, status_data);

    //filter the position and velocity
    std::pair<double, double> pos_and_velocity = joint_tmp->pos_filter.compute(
        actuator_state->position_unfiltered_, timestamp);
    //reset the position to the filtered value
    actuator_state->position_ = pos_and_velocity.first;
    //set the velocity to the filtered velocity
    actuator_state->velocity_ = pos_and_velocity.second;
  }

  template <class StatusType, class CommandType>
  sr_actuator::SrActuatorState* SrRobotLib<StatusType, CommandType>::get_joint_actuator_state(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp)
  {
    sr_actuator::SrActuatorState* actuator_state;

    if (sr_actuator::SrActuator* motor_actuator = static_cast<sr_actuator::SrActuator*>(joint_tmp->actuator_wrapper->actuator))
    {
      actuator_state = &motor_actuator->state_;
    }
    else if (sr_actuator::SrMuscleActuator* muscle_actuator = static_cast<sr_actuator::SrMuscleActuator*>(joint_tmp->actuator_wrapper->actuator))
    {
      actuator_state = &muscle_actuator->state_;
    }
    else
    {
      ROS_FATAL("Unknown actuator type. Known types: sr_actuator::SrActuator, sr_actuator::SrMuscleActuator");
      exit(EXIT_FAILURE);
    }

    return actuator_state;
  }

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::build_tactile_command(CommandType* command)
  {
    if (tactile_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      if (tactiles_init->sensor_updater->build_init_command(command)
          != operation_mode::device_update_state::INITIALIZATION)
      {
        tactile_current_state = operation_mode::device_update_state::OPERATION;

        switch (tactiles_init->tactiles_vector->at(0).which_sensor)
        {
        case TACTILE_SENSOR_PROTOCOL_TYPE_PST3:
          tactiles = boost::shared_ptr<tactiles::ShadowPSTs<StatusType, CommandType> >(
            new tactiles::ShadowPSTs<StatusType, CommandType>(pst3_sensor_update_rate_configs_vector,
                                     operation_mode::device_update_state::OPERATION,
                                     tactiles_init->tactiles_vector));

          ROS_INFO("PST3 tactiles initialized");
          break;

        case TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3:
          tactiles = boost::shared_ptr<tactiles::Biotac<StatusType, CommandType> >(
            new tactiles::Biotac<StatusType, CommandType>(biotac_sensor_update_rate_configs_vector, operation_mode::device_update_state::OPERATION,
                                 tactiles_init->tactiles_vector));

          ROS_INFO("Biotac tactiles initialized");
          break;

        case TACTILE_SENSOR_PROTOCOL_TYPE_UBI0:
          tactiles = boost::shared_ptr<tactiles::UBI0<StatusType, CommandType> >(
            new tactiles::UBI0<StatusType, CommandType>(ubi0_sensor_update_rate_configs_vector, operation_mode::device_update_state::OPERATION,
                               tactiles_init->tactiles_vector));

          ROS_INFO("UBI0 tactiles initialized");
          break;

        case TACTILE_SENSOR_PROTOCOL_TYPE_INVALID:
          ROS_WARN_STREAM("TACTILE_SENSOR_PROTOCOL_TYPE_INVALID!!");
          break;
        case TACTILE_SENSOR_PROTOCOL_TYPE_CONFLICTING:
          ROS_WARN_STREAM("TACTILE_SENSOR_PROTOCOL_TYPE_CONFLICTING!!");
          break;
        }
      }
    }
    else
      tactile_current_state = tactiles->sensor_updater->build_command(command);
  }

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::update_tactile_info(StatusType* status)
  {
    if (tactile_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      if( tactiles_init != NULL )
        tactiles_init->update(status);
    }
    else
    {
      if( tactiles != NULL )
        tactiles->update(status);
    }
  }

  template <class StatusType, class CommandType>
  shadow_joints::CalibrationMap SrRobotLib<StatusType, CommandType>::read_joint_calibration()
  {
    shadow_joints::CalibrationMap joint_calibration;
    std::string param_name = "sr_calibrations";

    XmlRpc::XmlRpcValue calib;
    nodehandle_.getParam(param_name, calib);
    ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      //check the calibration is well formatted:
      // first joint name, then calibration table
      ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string joint_name = static_cast<std::string> (calib[index_cal][0]);
      std::vector<joint_calibration::Point> calib_table_tmp;

      //now iterates on the calibration table for the current joint
      for(int32_t index_table=0; index_table < calib[index_cal][1].size(); ++index_table)
      {
        ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
        //only 2 values per calibration point: raw and calibrated (doubles)
        ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
        ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


        joint_calibration::Point point_tmp;
        point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
        point_tmp.calibrated_value = sr_math_utils::to_rad( static_cast<double> (calib[index_cal][1][index_table][1]) );
        calib_table_tmp.push_back(point_tmp);
      }

      joint_calibration.insert(joint_name, boost::shared_ptr<shadow_robot::JointCalibration>(new shadow_robot::JointCalibration(calib_table_tmp)) );
    }

    return joint_calibration;
  } //end read_joint_calibration

  template <class StatusType, class CommandType>
  std::vector<shadow_joints::JointToSensor> SrRobotLib<StatusType, CommandType>::read_joint_to_sensor_mapping()
  {
    std::vector<shadow_joints::JointToSensor> joint_to_sensor_vect;

    std::map<std::string, int> sensors_map;
    for(unsigned int i=0; i < SENSORS_NUM_0220; ++i)
    {
      sensors_map[ sensor_names[i] ] = i;
    }

    XmlRpc::XmlRpcValue joint_to_sensor_mapping;
    nodehandle_.getParam("joint_to_sensor_mapping", joint_to_sensor_mapping);
    ROS_ASSERT(joint_to_sensor_mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < joint_to_sensor_mapping.size(); ++i)
    {
      shadow_joints::JointToSensor tmp_vect;

      XmlRpc::XmlRpcValue map_one_joint = joint_to_sensor_mapping[i];

      //The parameter can either start by an array (sensor_name, coeff)
      // or by an integer to specify if we calibrate before combining
      // the different sensors
      int param_index = 0;
      //Check if the calibrate after combine int is set to 1
      if(map_one_joint[param_index].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        if(1 == static_cast<int>(map_one_joint[0]) )
          tmp_vect.calibrate_after_combining_sensors = true;
        else
          tmp_vect.calibrate_after_combining_sensors = false;

        param_index ++;
      }
      else //by default we calibrate before combining the sensors
        tmp_vect.calibrate_after_combining_sensors = false;

      ROS_ASSERT(map_one_joint.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = param_index; i < map_one_joint.size(); ++i)
      {
        ROS_ASSERT(map_one_joint[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        shadow_joints::PartialJointToSensor tmp_joint_to_sensor;

        ROS_ASSERT(map_one_joint[i][0].getType() == XmlRpc::XmlRpcValue::TypeString);
        tmp_vect.sensor_names.push_back( static_cast<std::string>(map_one_joint[i][0]) );
        tmp_joint_to_sensor.sensor_id = sensors_map[ static_cast<std::string>(map_one_joint[i][0]) ];

        ROS_ASSERT(map_one_joint[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        tmp_joint_to_sensor.coeff = static_cast<double> (map_one_joint[i][1]);
        tmp_vect.joint_to_sensor_vector.push_back(tmp_joint_to_sensor);
      }
      joint_to_sensor_vect.push_back(tmp_vect);
    }

    return joint_to_sensor_vect;
  } //end read_joint_to_sensor_mapping

  template <class StatusType, class CommandType>
  std::vector<generic_updater::UpdateConfig> SrRobotLib<StatusType, CommandType>::read_update_rate_configs(std::string base_param, int nb_data_defined, const char* human_readable_data_types[], const int32u data_types[])
  {
    std::vector<generic_updater::UpdateConfig> update_rate_configs_vector;
    typedef std::pair<std::string, int32u> ConfPair;
    std::vector<ConfPair> config;

    for(int i=0; i<nb_data_defined; ++i)
    {
      ConfPair tmp;

      ROS_DEBUG_STREAM(" read " << base_param << " config [" << i<< "] = "  << human_readable_data_types[i]);

      tmp.first = base_param + human_readable_data_types[i];
      tmp.second = data_types[i];
      config.push_back(tmp);
    }

    for( unsigned int i = 0; i < config.size(); ++i )
    {
      double rate;
      if (nodehandle_.getParam(config[i].first, rate))
      {
        generic_updater::UpdateConfig config_tmp;

        config_tmp.when_to_update = rate;
        config_tmp.what_to_update = config[i].second;
        update_rate_configs_vector.push_back(config_tmp);

        ROS_DEBUG_STREAM(" read " << base_param <<" config [" << i<< "] = "  << "what: "<< config_tmp.what_to_update << " when: " << config_tmp.when_to_update);
      }
    }

    return update_rate_configs_vector;
  }

  template <class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::checkSelfTests()
  {
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
      //check if we have some self diagnostics test to run and run them
      // in a separate thread
      self_tests_->checkTest();
      loop_rate.sleep();
    }
  }

  //Only to ensure that the template class is compiled for the types we are interested in
  template class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;
  template class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;
  template class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;

} //end namespace

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
