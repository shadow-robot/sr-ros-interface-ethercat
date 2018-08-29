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
#include <utility>
#include <map>
#include <vector>
#include <boost/foreach.hpp>

#include <sys/time.h>

#include <ros/ros.h>

#include "sr_robot_lib/shadow_PSTs.hpp"
#include "sr_robot_lib/biotac.hpp"
#include "sr_robot_lib/UBI0.hpp"
#include <controller_manager_msgs/ListControllers.h>
#include <sr_robot_lib/motor_updater.hpp>

#define SERIOUS_ERROR_FLAGS PALM_0200_EDC_SERIOUS_ERROR_FLAGS
#define error_flag_names palm_0200_edc_error_flag_names

using std::vector;
using std::string;
using std::pair;
using std::map;
using std::ostringstream;
using sr_actuator::SrMotorActuator;
using tactiles::GenericTactiles;
using tactiles::ShadowPSTs;
using tactiles::Biotac;
using tactiles::UBI0;
using ros_ethercat_model::RobotState;
using generic_updater::MotorUpdater;
using shadow_joints::CalibrationMap;
using shadow_joints::Joint;
using shadow_joints::JointToSensor;
using shadow_joints::MotorWrapper;
using shadow_joints::PartialJointToSensor;
using generic_updater::MotorUpdater;
using generic_updater::UpdateConfig;
using boost::shared_ptr;
using boost::ptr_vector;


namespace shadow_robot
{
#ifdef DEBUG_PUBLISHER
  // max of 20 publishers for debug
  template<class StatusType, class CommandType>
  const int SrRobotLib<StatusType, CommandType>::nb_debug_publishers_const = 20;
  // template <class StatusType, class CommandType>
  // const int SrRobotLib<StatusType, CommandType>::debug_mutex_lock_wait_time = 100;
#endif

  template <class StatusType, class CommandType>
  const int SrRobotLib<StatusType, CommandType>::nb_sensor_data = 37;

  template <class StatusType, class CommandType>
  const char *SrRobotLib<StatusType, CommandType>::human_readable_sensor_data_types[nb_sensor_data] =
      {
          "TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ",
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
          "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_20",
          "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_21",
          "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_22",
          "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_23",
          "TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_24",
          "TACTILE_SENSOR_TYPE_BIOTAC_PDC",
          "TACTILE_SENSOR_TYPE_BIOTAC_TAC",
          "TACTILE_SENSOR_TYPE_BIOTAC_TDC",
          "TACTILE_SENSOR_TYPE_UBI0_TACTILE"
        };

  template <class StatusType, class CommandType>
  const int32u SrRobotLib<StatusType, CommandType>::sensor_data_types[nb_sensor_data] =
  {
          TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ,
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
          TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_20,
          TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_21,
          TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_22,
          TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_23,
          TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_24,
          TACTILE_SENSOR_TYPE_BIOTAC_PDC,
          TACTILE_SENSOR_TYPE_BIOTAC_TAC,
          TACTILE_SENSOR_TYPE_BIOTAC_TDC,
          TACTILE_SENSOR_TYPE_UBI0_TACTILE
    };

  template<class StatusType, class CommandType>
  const double SrRobotLib<StatusType, CommandType>::tactile_timeout = 10.0;

  template<class StatusType, class CommandType>
  SrRobotLib<StatusType, CommandType>::SrRobotLib(hardware_interface::HardwareInterface *hw, ros::NodeHandle nh,
                                                  ros::NodeHandle nhtilde, string device_id, string joint_prefix)
          : main_pic_idle_time(0),
            main_pic_idle_time_min(1000),
            tactile_current_state(operation_mode::device_update_state::INITIALIZATION),
            hw_(static_cast<RobotState *> (hw)),
            device_id_(device_id),
            joint_prefix_(joint_prefix),
            nullify_demand_(false),
            nodehandle_(nh),
            nh_tilde(nhtilde),

            // advertise the service to nullify the demand sent to the motor
            // this makes it possible to easily stop the controllers.
            nullify_demand_server_(
                    nh_tilde.advertiseService("nullify_demand", &SrRobotLib::nullify_demand_callback, this)),

            // initialises self tests (false as this is not a simulated hand\)
            // self_tests_(new SrSelfTest(false)),
            //  self_test_thread_(new boost::thread(boost::bind(&SrRobotLib::checkSelfTests, this))),

            // read the generic sensor polling frequency from the parameter server
            generic_sensor_update_rate_configs_vector(
                    read_update_rate_configs("generic_sensor_data_update_rate/", nb_sensor_data,
                                             human_readable_sensor_data_types, sensor_data_types)),

            // read the pst3 sensor polling frequency from the parameter server
            pst3_sensor_update_rate_configs_vector(
                    read_update_rate_configs("pst3_sensor_data_update_rate/", nb_sensor_data,
                                             human_readable_sensor_data_types, sensor_data_types)),

            // read the biotac sensor polling frequency from the parameter server
            biotac_sensor_update_rate_configs_vector(
                    read_update_rate_configs("biotac_sensor_data_update_rate/", nb_sensor_data,
                                             human_readable_sensor_data_types, sensor_data_types)),

            // read the UBI0 sensor polling frequency from the parameter server
            ubi0_sensor_update_rate_configs_vector(
                    read_update_rate_configs("ubi0_sensor_data_update_rate/", nb_sensor_data,
                                             human_readable_sensor_data_types, sensor_data_types)),

            tactile_init_max_duration(tactile_timeout),

            // Create a one-shot timer
            tactile_check_init_timeout_timer(
                    this->nh_tilde.createTimer(tactile_init_max_duration,
                                               boost::bind(
                                                       &SrRobotLib<StatusType,
                                                               CommandType>::tactile_init_timer_callback,
                                                       this, _1), true)),
            lock_tactile_init_timeout_(boost::shared_ptr<boost::mutex>(new boost::mutex())),
            tactiles_init(shared_ptr<GenericTactiles<StatusType, CommandType> >(
                    new GenericTactiles<StatusType, CommandType>(nodehandle_, device_id_,
                                                                 generic_sensor_update_rate_configs_vector,
                                                                 operation_mode::device_update_state::INITIALIZATION))),

            // initialize the calibration map
            calibration_map(read_joint_calibration())
  {
  }

  template<class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::reinitialize_sensors()
  {
    // Create a new GenericTactiles object
    tactiles_init = shared_ptr<GenericTactiles<StatusType, CommandType> >(
            new GenericTactiles<StatusType, CommandType>(nodehandle_, device_id_,
                                                         generic_sensor_update_rate_configs_vector,
                                                         operation_mode::device_update_state::INITIALIZATION));
    tactile_current_state = operation_mode::device_update_state::INITIALIZATION;
  }

  template<class StatusType, class CommandType>
  bool SrRobotLib<StatusType, CommandType>::nullify_demand_callback(sr_robot_msgs::NullifyDemand::Request &request,
                                                                    sr_robot_msgs::NullifyDemand::Response &response)
  {
    if (request.nullify_demand)
      ROS_INFO_STREAM(
              "Nullifying the demand sent to the motor. "
                      "Will ignore the values computed by the controllers and send 0.");
    else
      ROS_INFO_STREAM("Using the value computed by the controllers to send the demands to the motors.");

    nullify_demand_ = request.nullify_demand;
    return true;
  }

  template<class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::build_tactile_command(CommandType *command)
  {
    // Mutual exclusion with the the initialization timeout
    boost::mutex::scoped_lock l(*lock_tactile_init_timeout_);

    if (tactile_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      if (tactiles_init->sensor_updater->build_init_command(command)
          != operation_mode::device_update_state::INITIALIZATION)
      {
        tactile_current_state = operation_mode::device_update_state::OPERATION;

        tactile_check_init_timeout_timer.stop();

        switch (tactiles_init->tactiles_vector->at(0).which_sensor)
        {
          case TACTILE_SENSOR_PROTOCOL_TYPE_PST3:
            tactiles = shared_ptr<ShadowPSTs<StatusType, CommandType> >(
                    new ShadowPSTs<StatusType, CommandType>(nodehandle_, device_id_,
                                                            pst3_sensor_update_rate_configs_vector,
                                                            operation_mode::device_update_state::OPERATION,
                                                            tactiles_init->tactiles_vector));

            ROS_INFO("PST3 tactiles initialized");
            break;

          case TACTILE_SENSOR_PROTOCOL_TYPE_BIOTAC_2_3:
            tactiles = shared_ptr<Biotac<StatusType, CommandType> >(
                    new Biotac<StatusType, CommandType>(nodehandle_, device_id_,
                                                        biotac_sensor_update_rate_configs_vector,
                                                        operation_mode::device_update_state::OPERATION,
                                                        tactiles_init->tactiles_vector));

            ROS_INFO("Biotac tactiles initialized");
            break;

          case TACTILE_SENSOR_PROTOCOL_TYPE_UBI0:
            tactiles = shared_ptr<UBI0<StatusType, CommandType> >(
                    new UBI0<StatusType, CommandType>(nodehandle_, device_id_, ubi0_sensor_update_rate_configs_vector,
                                                      operation_mode::device_update_state::OPERATION,
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
    {
      tactile_current_state = tactiles->sensor_updater->build_command(command);
    }
  }

  template<class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::update_tactile_info(StatusType *status)
  {
    // Mutual exclusion with the the initialization timeout
    boost::mutex::scoped_lock l(*lock_tactile_init_timeout_);
    if (tactile_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      if (tactiles_init != NULL)
      {
        tactiles_init->update(status);
      }
    }
    else
    {
      if (tactiles != NULL)
      {
        tactiles->update(status);
      }
    }
  }

  template<class StatusType, class CommandType>
  CalibrationMap SrRobotLib<StatusType, CommandType>::read_joint_calibration()
  {
    CalibrationMap joint_calibration;

    XmlRpc::XmlRpcValue calib;
    nodehandle_.getParam("sr_calibrations", calib);
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
        point_tmp.calibrated_value = sr_math_utils::to_rad(static_cast<double> (calib[index_cal][1][index_table][1]));
        calib_table_tmp.push_back(point_tmp);
      }

      joint_calibration.insert(joint_name, shared_ptr<shadow_robot::JointCalibration>(
              new shadow_robot::JointCalibration(calib_table_tmp)));
    }

    return joint_calibration;
  }  // end read_joint_calibration

  template<class StatusType, class CommandType>
  vector<JointToSensor> SrRobotLib<StatusType, CommandType>::read_joint_to_sensor_mapping()
  {
    vector<JointToSensor> joint_to_sensor_vect;

    map<string, int> sensors_map;
    for (unsigned int i = 0; i < SENSORS_NUM_0220; ++i)
    {
      sensors_map[sensor_names[i]] = i;
    }

    XmlRpc::XmlRpcValue joint_to_sensor_mapping;
    nodehandle_.getParam("joint_to_sensor_mapping", joint_to_sensor_mapping);
    ROS_ASSERT(joint_to_sensor_mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < joint_to_sensor_mapping.size(); ++i)
    {
      JointToSensor tmp_vect;

      XmlRpc::XmlRpcValue map_one_joint = joint_to_sensor_mapping[i];

      // The parameter can either start by an array (sensor_name, coeff)
      // or by an integer to specify if we calibrate before combining
      // the different sensors
      int param_index = 0;
      // Check if the calibrate after combine int is set to 1
      if (map_one_joint[param_index].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        if (1 == static_cast<int> (map_one_joint[0]))
        {
          tmp_vect.calibrate_after_combining_sensors = true;
        }
        else
        {
          tmp_vect.calibrate_after_combining_sensors = false;
        }

        param_index++;
      }
      else
      {
        // by default we calibrate before combining the sensors
        tmp_vect.calibrate_after_combining_sensors = false;
      }

      ROS_ASSERT(map_one_joint.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = param_index; i < map_one_joint.size(); ++i)
      {
        ROS_ASSERT(map_one_joint[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        PartialJointToSensor tmp_joint_to_sensor;

        ROS_ASSERT(map_one_joint[i][0].getType() == XmlRpc::XmlRpcValue::TypeString);
        tmp_vect.sensor_names.push_back(static_cast<string> (map_one_joint[i][0]));
        tmp_joint_to_sensor.sensor_id = sensors_map[static_cast<string> (map_one_joint[i][0])];

        ROS_ASSERT(map_one_joint[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        tmp_joint_to_sensor.coeff = static_cast<double> (map_one_joint[i][1]);
        tmp_vect.joint_to_sensor_vector.push_back(tmp_joint_to_sensor);
      }
      joint_to_sensor_vect.push_back(tmp_vect);
    }

    return joint_to_sensor_vect;
  }  // end read_joint_to_sensor_mapping

  template<class StatusType, class CommandType>
  vector<UpdateConfig> SrRobotLib<StatusType,
          CommandType>::read_update_rate_configs(string base_param,
                                                 int nb_data_defined,
                                                 const char *human_readable_data_types[],
                                                 const int32u data_types[])
  {
    vector<UpdateConfig> update_rate_configs_vector;
    typedef pair<string, int32u> ConfPair;
    vector<ConfPair> config;

    for (int i = 0; i < nb_data_defined; ++i)
    {
      ConfPair tmp;

      ROS_DEBUG_STREAM(" read " << base_param << " config [" << i << "] = " << human_readable_data_types[i]);

      tmp.first = base_param + human_readable_data_types[i];
      tmp.second = data_types[i];
      config.push_back(tmp);
    }

    for (unsigned int i = 0; i < config.size(); ++i)
    {
      double rate;
      if (nodehandle_.getParam(config[i].first, rate))
      {
        UpdateConfig config_tmp;

        config_tmp.when_to_update = rate;
        config_tmp.what_to_update = config[i].second;
        update_rate_configs_vector.push_back(config_tmp);

        ROS_DEBUG_STREAM(
                " read " << base_param << " config [" << i << "] = " << "what: " << config_tmp.what_to_update <<
                " when: " << config_tmp.when_to_update);
      }
    }

    return update_rate_configs_vector;
  }

  template<class StatusType, class CommandType>
  void SrRobotLib<StatusType, CommandType>::tactile_init_timer_callback(const ros::TimerEvent &event)
  {
    // Mutual exclusion with the the initialization timeout
    boost::mutex::scoped_lock l(*lock_tactile_init_timeout_);

    if (tactile_current_state == operation_mode::device_update_state::INITIALIZATION)
    {
      tactile_current_state = operation_mode::device_update_state::OPERATION;
      tactiles = boost::shared_ptr<tactiles::UBI0<StatusType, CommandType> >(
              new tactiles::UBI0<StatusType, CommandType>(nodehandle_, device_id_,
                                                          ubi0_sensor_update_rate_configs_vector,
                                                          operation_mode::device_update_state::OPERATION,
                                                          tactiles_init->tactiles_vector));
      ROS_ERROR_STREAM("Tactile Initialization Timeout: considering UBI0 tactiles");
    }
  }

// Only to ensure that the template class is compiled for the types we are interested in
  template
  class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;

  template
  class SrRobotLib<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;

}  // namespace shadow_robot

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
