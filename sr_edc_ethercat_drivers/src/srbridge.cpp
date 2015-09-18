/**
 * @file   srbridge.cpp
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 11:35:21 2011
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
 * @brief This is a ROS driver for the etherCAT bridge.
 */

#include <sr_edc_ethercat_drivers/srbridge.h>

#include <iomanip>
#include <sstream>
#include <cassert>

PLUGINLIB_EXPORT_CLASS(SRBridge, EthercatDevice);

void SRBridge::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh, start_address);

  assert(sh_->get_product_code() == PRODUCT_CODE);

  ROS_INFO("Shadow Bridge configure -  %d @ %d", sh_->get_product_code(), sh_->get_ring_position());
}

int SRBridge::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  SR0X::initialize(hw, allow_unprogrammed);

  assert(sh_->get_product_code() == PRODUCT_CODE);

  EthercatDirectCom com(sh_->m_router_instance->m_al_instance->m_dll_instance);
  uint16_t data, new_data;
  int rv;

  rv = readData(&com, (uint16_t) EC_Slave_RD[PDI_Conf_reg].ado, &data, 2, FIXED_ADDR);
  ROS_INFO("bridge port type: %s", data & 1 ? "MII" : "EBUS");

  rv = readData(&com, 0x100, &data, 2, FIXED_ADDR);
  if (rv != 0)
    ROS_ERROR("can't read open status");

  new_data = data & ~0xc000;

  rv = writeData(&com, 0x100, &new_data, 2, FIXED_ADDR);
  if (rv != 0)
    ROS_ERROR("can't write DL values");

  rv = readData(&com, 0x100, &data, 2, FIXED_ADDR);
  if (rv != 0)
    ROS_ERROR("can't read open status");

  return 0;
}

void SRBridge::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  std::ostringstream str;
  str << "SRBridge : " << std::setw(2) << std::setfill('0') << sh_->get_ring_position();
  d.name = str.str();
  str.str("");
  str << sh_->get_product_code() << '-' << sh_->get_serial();
  d.hardware_id = str.str();

  d.message = "";
  d.level = 0;

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code", "%08x", sh_->get_product_code());
  d.addf("Serial", "%08x", sh_->get_serial());
  d.addf("Revision", "%08x", sh_->get_revision());

  this->ethercatDiagnostics(d, 2);
}
