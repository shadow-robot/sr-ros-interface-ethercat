/**
 * @file   sr_self_test.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Feb 4, 2013
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
 * @brief Class containing the self tests for the Shadow Robot EtherCAT hardware.
 *
 *
 */

#ifndef SR_SELF_TEST_HPP_
#define SR_SELF_TEST_HPP_

#include "diagnostic_msgs/SelfTest.h"
#include "self_test/self_test.h"
#include "sr_robot_lib/sr_robot_lib.hpp"

namespace shadow_robot
{
//forward declaration of SrRobotLib
class SrRobotLib;

class SrSelfTest {
public:
  SrSelfTest(SrRobotLib* robot_lib);
  ~SrSelfTest();

  void first_test(diagnostic_updater::DiagnosticStatusWrapper& status);
private:
  SrRobotLib* robot_lib_;

  // self_test::TestRunner is the handles sequencing driver self-tests.
  self_test::TestRunner test_runner_;
};
}

#endif /* SR_SELF_TEST_HPP_ */
