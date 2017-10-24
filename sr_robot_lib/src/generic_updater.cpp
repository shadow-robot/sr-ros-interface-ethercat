/**
 * @file   generic_updater.cpp
 * @author toni <toni@shadowrobot.com>
 * @date   20 Oct 2011
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
 * @brief This is a generic command updater: it has 2 different queues one for
 *        the important data which is updated as fast as possible, one for
 *        unimportant data which is updated on a time basis.
 *
 *
 */

#include "sr_robot_lib/generic_updater.hpp"
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>

namespace generic_updater
{
  template<class CommandType>
  GenericUpdater<CommandType>::GenericUpdater(std::vector<UpdateConfig> update_configs_vector,
                                              operation_mode::device_update_state::DeviceUpdateState update_state)
          : nh_tilde("~"), which_data_to_request(0), update_state(update_state), update_configs_vector(
          update_configs_vector)
  {
    mutex = boost::shared_ptr<boost::mutex>(new boost::mutex());

    BOOST_FOREACH(UpdateConfig config, update_configs_vector)
          {
            if (config.when_to_update == -2.0)
            {
              initialization_configs_vector.push_back(config);
            }
            else if (config.when_to_update != -1.0)
            {
              double tmp_dur = config.when_to_update;
              ros::Duration duration(tmp_dur);

              timers.push_back(
                      nh_tilde.createTimer(
                              duration,
                              boost::bind(&GenericUpdater::timer_callback, this, _1, config.what_to_update)));
            }
            else
            {
              important_update_configs_vector.push_back(config);
            }
          }
    ROS_DEBUG_STREAM("Init config size" << initialization_configs_vector.size());
    ROS_DEBUG_STREAM("Important config size" << important_update_configs_vector.size());

    // If there isn't any command defined as initializingcommand (-2), state switches to operation
    if (initialization_configs_vector.size() == 0)
    {
      ROS_INFO_STREAM("No init command. Switching to operation");
      update_state = operation_mode::device_update_state::OPERATION;
    }
  }

  template<class CommandType>
  void GenericUpdater<CommandType>::timer_callback(const ros::TimerEvent &event, int32u data_type)
  {
    if (update_state == operation_mode::device_update_state::OPERATION)
    {
      boost::mutex::scoped_lock l(*mutex);
      unimportant_data_queue.push(data_type);

      ROS_DEBUG_STREAM("Timer: data type = " << data_type << " | queue size: " << unimportant_data_queue.size());
    }
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class GenericUpdater<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class GenericUpdater<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class GenericUpdater<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;

  template
  class GenericUpdater<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}  // end namespace generic_updater

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */
