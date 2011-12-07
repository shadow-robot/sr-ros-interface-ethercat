/**
 * @file   tactile_sensors.cpp
 * @author toni <toni@shadowrobot.com>
 * @date   21 Oct 2011
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

#include "sr_robot_lib/tactile_sensors.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <sstream>

namespace tactiles
{

  void GenericTactileData::set_software_version( std::string version )
  {
    //split the string to fill the different versions
    std::vector<std::string> splitted_string;
    boost::split(splitted_string, version, boost::is_any_of("\n "));

    BOOST_ASSERT(splitted_string.size() == 3);
    software_version_current = convertToInt(splitted_string[0]);
    software_version_server = convertToInt(splitted_string[1]);

    if( splitted_string[2] == "No")
      software_version_modified = false;
    else
      software_version_modified = true;
  }

  std::string GenericTactileData::get_software_version()
  {
    //concatenate versions in a string.
    std::string full_version;

    std::stringstream ss;
    if( software_version_modified )
      ss << "current: " << software_version_current << " / server: " << software_version_server << " / MODIFIED";
    else
      ss << "current: " << software_version_current << " / server: " << software_version_server << " / not modified";

    full_version = ss.str();

    return full_version;
  }

  PST3Data::PST3Data()
    : GenericTactileData()
  {};

  PST3Data::PST3Data(const PST3Data& pst3)
    : GenericTactileData(pst3.tactile_data_valid, pst3.sample_frequency,
                         pst3.manufacturer, pst3.serial_number,
                         pst3.software_version_current,
                         pst3.software_version_server,
                         pst3.software_version_modified,
                         pst3.pcb_version),
      pressure(pst3.pressure), temperature(pst3.temperature),
      debug_1(pst3.debug_1), debug_2(pst3.debug_2),
      pressure_raw(pst3.pressure_raw), zero_tracking(pst3.zero_tracking), dac_value(pst3.dac_value)
  {};

  PST3Data::PST3Data(const GenericTactileData& gtd)
    : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                         gtd.manufacturer, gtd.serial_number,
                         gtd.software_version_current,
                         gtd.software_version_server,
                         gtd.software_version_modified,
                         gtd.pcb_version)
  {};

  BiotacData::BiotacData()
    : GenericTactileData()
  {};

  BiotacData::BiotacData(const BiotacData& btac)
    : GenericTactileData(btac.tactile_data_valid, btac.sample_frequency,
                         btac.manufacturer, btac.serial_number,
                         btac.software_version_current,
                         btac.software_version_server,
                         btac.software_version_modified,
                         btac.pcb_version),
      pac0(btac.pac0), pac1(btac.pac1),
      pdc(btac.pdc), tac(btac.tac),
      tdc(btac.tdc)
  {
    for(unsigned int i =0; i<btac.electrodes.size() ;i++)
    {
      electrodes[i] = btac.electrodes[i];
    }
  };

  BiotacData::BiotacData(const GenericTactileData& gtd)
    : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                         gtd.manufacturer, gtd.serial_number,
                         gtd.software_version_current,
                         gtd.software_version_server,
                         gtd.software_version_modified,
                         gtd.pcb_version)
  {};
}




/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
