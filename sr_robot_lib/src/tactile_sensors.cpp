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

namespace tactiles
{

      PST3Data::PST3Data() {};

      PST3Data::PST3Data(const PST3Data& pst3)
      : tactile_data_valid(pst3.sample_frequency), sample_frequency(pst3.sample_frequency),
        manufacturer(pst3.manufacturer), serial_number(pst3.serial_number),
        software_version(pst3.software_version), pcb_version(pst3.pcb_version),
        pressure(pst3.pressure), temperature(pst3.temperature),
        debug_1(pst3.debug_1), debug_2(pst3.debug_2),
        pressure_raw(pst3.pressure_raw), zero_tracking(pst3.zero_tracking), dac_value(pst3.dac_value)
      {};

      PST3Data::PST3Data(const GenericTactileData& gtd)
        : tactile_data_valid(gtd.sample_frequency), sample_frequency(gtd.sample_frequency),
          manufacturer(gtd.manufacturer), serial_number(gtd.serial_number),
          software_version(gtd.software_version), pcb_version(gtd.pcb_version)
        {};

      BiotacData::BiotacData() {};

      BiotacData::BiotacData(const BiotacData& btac)
      : tactile_data_valid(btac.sample_frequency), sample_frequency(btac.sample_frequency),
        manufacturer(btac.manufacturer), serial_number(btac.serial_number),
        software_version(btac.software_version), pcb_version(btac.pcb_version),
        pac0(btac.pac0), pac1(btac.pac1),
        pdc(btac.pdc), tac(btac.tac),
        tdc(btac.tdc)
      {
        for(unsigned int i;i<btac.electrodes.size();i++)
        {
          electrodes[i] = btac.electrodes[i];
        }
      };

      BiotacData::BiotacData(const GenericTactileData& gtd)
        : tactile_data_valid(gtd.sample_frequency), sample_frequency(gtd.sample_frequency),
          manufacturer(gtd.manufacturer), serial_number(gtd.serial_number),
          software_version(gtd.software_version), pcb_version(gtd.pcb_version)
        {};
}




/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
