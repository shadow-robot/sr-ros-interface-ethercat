/**
 * @file   test_robot_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jun 22 13:04:41 2011
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
 * @brief This is a set of unit tests testing the robot libraries.
 *
 *
 */

#include "sr_robot_lib/sr_motor_hand_lib.hpp"
#include <sr_mechanism_model/simple_transmission.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <utility>
#include <string>
#include <vector>

#define error_flag_names palm_0200_edc_error_flag_names
#define STATUS_TYPE ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS
#define COMMAND_TYPE ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND

class HandLibTestProtected :
        public shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE>
{
public:
  HandLibTestProtected(hardware_interface::HardwareInterface *hw, ros::NodeHandle nh)
          : shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE>(hw, nh, nh, "", "rh_")
  {
  };

public:
  using shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE>::joints_vector;
  using shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE>::humanize_flags;
};

class HandLibTest
{
  ros::NodeHandle nh;
public:
  ros_ethercat_model::RobotState *hw = NULL;
  boost::shared_ptr<HandLibTestProtected> sr_hand_lib;
  sr_actuator::SrMotorActuator *actuator;
  XmlRpc::XmlRpcValue joint_to_sensor_mapping;

  HandLibTest()
  {
    TiXmlElement *root;
    TiXmlElement *root_element;
    TiXmlDocument xml;
    std::string robot_description;
    if (ros::param::get("/robot_description", robot_description))
    {
      xml.Parse(robot_description.c_str());
    }
    else
    {
      ROS_ERROR_STREAM("Could not load the xml from parameter server");
    }
    root_element = xml.RootElement();
    root = xml.FirstChildElement("robot");

    nh.getParam("joint_to_sensor_mapping", joint_to_sensor_mapping);
    hw = new ros_ethercat_model::RobotState(root);

    hardware_interface::HardwareInterface *ehw = static_cast<hardware_interface::HardwareInterface *>(hw);
    sr_hand_lib.reset(new HandLibTestProtected(ehw, nh));
  }

  ~HandLibTest()
  {
    delete hw;
  }

  void check_hw_actuator(std::string name, int motor_id, int id_in_enum, double expected_pos)
  {
    actuator = static_cast<sr_actuator::SrMotorActuator *>(hw->getActuator(name));

    EXPECT_EQ(actuator->state_.device_id_, motor_id);
    // EXPECT_EQ(state.position_ , expected_pos);
  }
};

/**
 * Tests the initialization of the hand library.
 */
TEST(SrRobotLib, Initialization)
{
  boost::shared_ptr<HandLibTest> lib_test = boost::shared_ptr<HandLibTest>(new HandLibTest());

//  pr2_hardware_interface::HardwareInterface *hw;
//  hw = new pr2_hardware_interface::HardwareInterface();
//  boost::shared_ptr< shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE> > lib_test =
// boost::shared_ptr< shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE> >(
// new shadow_robot::SrMotorHandLib<STATUS_TYPE, COMMAND_TYPE>(hw) );

  EXPECT_TRUE(true);
}

/**
 * Tests the update of the hand library.
 */
TEST(SrRobotLib, UpdateMotor)
{
  boost::shared_ptr<HandLibTest> lib_test = boost::shared_ptr<HandLibTest>(new HandLibTest());

  STATUS_TYPE *status_data = new STATUS_TYPE();
  // add growing sensors values
  for (unsigned int i = 1; i < SENSORS_NUM_0220 + 2; ++i)
  {
    // position = id in joint enum
    status_data->sensors[i] = i;
  }

  status_data->motor_data_type = MOTOR_DATA_SGR;

  // even motors
  status_data->which_motors = 0;

  // all motor data arrived with no errors
  status_data->which_motor_data_arrived = 0x00055555;
  status_data->which_motor_data_had_errors = 0;

  // add growing motor data packet values
  for (unsigned int i = 0; i < 10; ++i)
  {
    status_data->motor_data_packet[i].torque = 4;
    status_data->motor_data_packet[i].misc = 2 * i;
  }
  // filling the status data with known values
  status_data->idle_time_us = 1;

  // update the library
  lib_test->sr_hand_lib->update(status_data);

  // check the data we read back are correct.
  EXPECT_EQ(lib_test->sr_hand_lib->main_pic_idle_time, 1);
  EXPECT_EQ(lib_test->sr_hand_lib->main_pic_idle_time_min, 1);

  // check the sensors etc..
  std::vector<shadow_joints::Joint>::iterator joint_tmp = lib_test->sr_hand_lib->joints_vector.begin();
  for (; joint_tmp != lib_test->sr_hand_lib->joints_vector.end(); ++joint_tmp)
  {
    if (joint_tmp->has_actuator)
    {
      boost::shared_ptr<shadow_joints::MotorWrapper> motor_wrapper =
              boost::static_pointer_cast<shadow_joints::MotorWrapper>(joint_tmp->actuator_wrapper);
      // we updated the even motors
      if (motor_wrapper->motor_id % 2 == 0)
      {
        const sr_actuator::SrMotorActuator *sr_actuator =
                static_cast<sr_actuator::SrMotorActuator *>(motor_wrapper->actuator);

        ROS_ERROR_STREAM("last measured effort: " << sr_actuator->state_.last_measured_effort_ << " actuator: " <<
                         motor_wrapper->actuator);

        EXPECT_FLOAT_EQ(sr_actuator->motor_state_.force_unfiltered_, 4.0);  // (double)motor_wrapper->motor_id/2.0);
        EXPECT_EQ(sr_actuator->motor_state_.strain_gauge_right_, motor_wrapper->motor_id);
      }
    }
  }
  delete status_data;
}

/**
 * Tests the update of the actuators
 * which are in the hw*
 */

TEST(SrRobotLib, UpdateActuators)
{
  boost::shared_ptr<HandLibTest> lib_test = boost::shared_ptr<HandLibTest>(new HandLibTest());

  STATUS_TYPE *status_data = new STATUS_TYPE();
  // add growing sensors values
  for (unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
  {
    // position = id in joint enum
    status_data->sensors[i] = i + 1;
  }

  status_data->motor_data_type = MOTOR_DATA_VOLTAGE;

  // even motors
  status_data->which_motors = 0;

  // all motor data arrived with no errors
  status_data->which_motor_data_arrived = 0x00055555;
  status_data->which_motor_data_had_errors = 0;

  // add growing motor data packet values
  for (unsigned int i = 0; i < 10; ++i)
  {
    status_data->motor_data_packet[i].torque = i;
    status_data->motor_data_packet[i].misc = 10 * i;
  }
  // filling the status data with known values
  status_data->idle_time_us = 1;

  // update the library
  lib_test->sr_hand_lib->update(status_data);

  // name, motor_id, id_in_enum, expected_pos
  lib_test->check_hw_actuator("rh_FFJ4", 2, 3, 4.0);

  // cleanup
  delete status_data;
}

/**
 * For the next tests we want to have access to the calibrate_joint
 * method which is protected in our code.
 * The method is found at:
 * http://code.google.com/p/googletest/wiki/V1_6_FAQ#How_do_I_test_private_class_members_without_writing_FRIEND_TEST(
 */
class TestHandLib
        :
                public HandLibTestProtected
{
public:
  TestHandLib(hardware_interface::HardwareInterface *hw, ros::NodeHandle nh)
          : HandLibTestProtected(hw, nh)
  {
  }

  using HandLibTestProtected::calibrate_joint;

  // using HandLibTestProtected::status_data;

  // using HandLibTestProtected::actuator;

  using HandLibTestProtected::humanize_flags;
};


/**
 * Testing the calibration procedure for
 * a joint having one motor only (FFJ3)
 *
 */
// TEST(SrRobotLib, CalibrationOneMotor)
// {

//   pr2_hardware_interface::HardwareInterface *hw;
//   boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

//   STATUS_TYPE status_data;

//  // set all the sensors to 0
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     status_data.sensors[i] = 0;

//   sr_hand_lib->status_data = &status_data;
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

//  // let's find FFJ3 in the vector
//   boost::ptr_vector<shadow_joints::Joint>::iterator ffj3 = sr_hand_lib->joints_vector.begin();
//   std::string name_tmp = ffj3->joint_name;
//   bool ffj3_found = false;
//   int index_ffj3 = 0;

//   for(; ffj3 != sr_hand_lib->joints_vector.end(); ++ffj3)
//   {
//     name_tmp = ffj3->joint_name;

//     if( name_tmp.compare("FFJ3") == 0)
//     {
//       ffj3_found = true;
//       break;
//     }

//     ++index_ffj3;
//   }

//   EXPECT_TRUE(ffj3_found);
//   EXPECT_EQ(index_ffj3, 3);

//   sr_hand_lib->actuator = (ffj3->motor->actuator);

//   sr_hand_lib->calibrate_joint(ffj3);
//  // all the sensors at 0 -> should be 0
//   EXPECT_EQ( ffj3->motor->actuator->state_.position_unfiltered_ , 0.0);

//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//   sr_hand_lib->status_data->sensors[i] = 1;

//  // now ffj3 position should be 1
//   sr_hand_lib->calibrate_joint(ffj3);
//  // all the sensors at 1 -> should be 1
//   EXPECT_EQ( ffj3->motor->actuator->state_.position_unfiltered_ , 1.0);

//   delete hw;
// }


// /**
//  * Testing the calibration procedure for
//  * a joint having calibrating the sensors first
//  * and then combining them (FFJ0)
//  *
//  */
// TEST(SrRobotLib, CalibrationFFJ0)
// {

//   pr2_hardware_interface::HardwareInterface *hw;
//   boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

//   STATUS_TYPE status_data;

//  // set all the sensors to 0
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     status_data.sensors[i] = 0;

//   sr_hand_lib->status_data = &status_data;
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

//  // let's find FFJ0 in the vector
//   boost::ptr_vector<shadow_joints::Joint>::iterator ffj0 = sr_hand_lib->joints_vector.begin();
//   std::string name_tmp = ffj0->joint_name;
//   bool ffj0_found = false;
//   int index_ffj0 = 0;

//   for(; ffj0 != sr_hand_lib->joints_vector.end(); ++ffj0)
//   {
//     name_tmp = ffj0->joint_name;

//     if( name_tmp.compare("FFJ0") == 0)
//     {
//       ffj0_found = true;
//       break;
//     }

//     ++index_ffj0;
//   }

//   EXPECT_TRUE(ffj0_found);
//   EXPECT_EQ(index_ffj0, 0);

//   sr_hand_lib->actuator = (ffj0->motor->actuator);

//   sr_hand_lib->calibrate_joint(ffj0);
//  // all the sensors at 0 -> should be 0
//   EXPECT_EQ( ffj0->motor->actuator->state_.position_unfiltered_ , 0.0);

//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     sr_hand_lib->status_data->sensors[i] = 1;

//  // now ffj0 position should be 1
//   sr_hand_lib->calibrate_joint(ffj0);
//  // all the sensors at 1 -> should be 2
//   EXPECT_EQ( ffj0->motor->actuator->state_.position_unfiltered_ , 2.0);

//   delete hw;
// }

// /**
//  * Testing the calibration procedure for
//  * a compound joint combining the sensors
//  * and then calibrating the total (THJ5)
//  *
//  */
// TEST(SrRobotLib, CalibrationTHJ5)
// {

//   pr2_hardware_interface::HardwareInterface *hw;
//   boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

//   STATUS_TYPE status_data;

//  // set all the sensors to 0
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     status_data.sensors[i] = 0;

//   sr_hand_lib->status_data = &status_data;
//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

//  // let's find THJ5 in the vector
//   boost::ptr_vector<shadow_joints::Joint>::iterator thj5 = sr_hand_lib->joints_vector.begin();
//   std::string name_tmp = thj5->joint_name;
//   bool thj5_found = false;
//   int index_thj5 = 0;

//   for(; thj5 != sr_hand_lib->joints_vector.end(); ++thj5)
//   {
//     name_tmp = thj5->joint_name;

//     if( name_tmp.compare("THJ5") == 0)
//     {
//       thj5_found = true;
//       break;
//     }

//     ++index_thj5;
//   }

//   EXPECT_TRUE(thj5_found);
//   EXPECT_EQ(index_thj5, 25);

//   sr_hand_lib->actuator = (thj5->motor->actuator);

//   sr_hand_lib->calibrate_joint(thj5);
//  // all the sensors at 0 -> should be 0
//   EXPECT_EQ( thj5->motor->actuator->state_.position_unfiltered_ , 0.0);

//   for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
//     sr_hand_lib->status_data->sensors[i] = 1;

//  // now thj5 position should be 1
//   sr_hand_lib->calibrate_joint(thj5);
//  // all the sensors at 1 -> should be 1 (THJ5 = .5 THJ5A + .5 THJ5B)
//   EXPECT_EQ( thj5->motor->actuator->state_.position_unfiltered_ , 1.0);

//   delete hw;
// }

/**
 * Testing the humanization of the flags.
 *
 */
TEST(SrRobotLib, HumanizeFlags)
{
  boost::shared_ptr<HandLibTest> lib_test = boost::shared_ptr<HandLibTest>(new HandLibTest());

  std::vector<std::pair<std::string, bool> > flags;
  // all flags set
  flags = lib_test->sr_hand_lib->humanize_flags(0xFFFF);

  EXPECT_EQ(flags.size(), 16);

  for (unsigned int i = 0; i < 16; ++i)
  {
    EXPECT_EQ(flags[i].first.compare(error_flag_names[i]), 0);
  }

  // The last three flags are serious
  EXPECT_TRUE(flags[13].second);
  EXPECT_TRUE(flags[14].second);
  EXPECT_TRUE(flags[15].second);
}

/////////////////////
//     MAIN       //
///////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

