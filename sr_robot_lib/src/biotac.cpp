/* Copyright 2011, 2023-2024 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/
/**
  * @file   biotac.hpp
  * @author Ugo Cupcic <ugo@shadowrobot.com>
  * @brief This is a class for accessing the data from the
  *        Biotac tactiles.
  */

#include "sr_robot_lib/biotac.hpp"
#include <sr_utilities/sr_math_utils.hpp>
#include <string>
#include <vector>

#define TACTILE_DATA_LENGTH_BYTES TACTILE_DATA_LENGTH_BYTES_v1

namespace tactiles
{

  template <class StatusType, class CommandType>
  const size_t Biotac<StatusType, CommandType>::nb_electrodes_v1_ = 19;

  template <class StatusType, class CommandType>
  const size_t Biotac<StatusType, CommandType>::nb_electrodes_v2_ = 24;

  template<class StatusType, class CommandType>
  Biotac<StatusType, CommandType>::Biotac(ros::NodeHandle nh, std::string device_id,
                                          std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                          operation_mode::device_update_state::DeviceUpdateState update_state)
          : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
  }

  template<class StatusType, class CommandType>
  Biotac<StatusType, CommandType>::Biotac(ros::NodeHandle nh, std::string device_id,
                                          std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                          operation_mode::device_update_state::DeviceUpdateState update_state,
                                          boost::shared_ptr<std::vector<GenericTactileData> > init_tactiles_vector)
          : GenericTactiles<StatusType, CommandType>(nh, device_id, update_configs_vector, update_state)
  {
    init(update_configs_vector, update_state);
    for (unsigned int i = 0; i < this->nb_tactiles; i++)
    {
      BiotacData tmp_biotac(init_tactiles_vector->at(i));
      this->all_tactile_data->at(i).biotac = tmp_biotac;
    }

    set_version_specific_details();
  }

  template<class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::init(std::vector<generic_updater::UpdateConfig> update_configs_vector,
                                             operation_mode::device_update_state::DeviceUpdateState update_state)
  {
    // initialize the vector of tactiles
    this->all_tactile_data = boost::shared_ptr<std::vector<AllTactileData> >(
            new std::vector<AllTactileData>(this->nb_tactiles));

    for (size_t i = 0; i < this->all_tactile_data->size(); ++i)
    {
      this->all_tactile_data->at(i).type = "biotac";
    }
  }

  template<class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::update(StatusType *status_data)
  {
    int tactile_mask = static_cast<int16u>(status_data->tactile_data_valid);
    // @todo use memcopy instead?
    for (unsigned int id_sensor = 0; id_sensor < this->nb_tactiles; ++id_sensor)
    {
      TACTILE_SENSOR_BIOTAC_DATA_CONTENTS *tactile_data =
        reinterpret_cast<TACTILE_SENSOR_BIOTAC_DATA_CONTENTS*> (&(status_data->tactile[id_sensor]));
      // We always receive two latest Pac values
      this->all_tactile_data->at(id_sensor).biotac.pac0 = static_cast<int>(tactile_data->Pac[0]);
      this->all_tactile_data->at(id_sensor).biotac.pac1 = static_cast<int>(tactile_data->Pac[1]);
      this->all_tactile_data->at(id_sensor).biotac.pac_buffer_.push_back(static_cast<int>(tactile_data->Pac[0]));
      this->all_tactile_data->at(id_sensor).biotac.pac_buffer_.push_back(static_cast<int>(tactile_data->Pac[1]));

      // the rest of the data is sampled at different rates
      switch (static_cast<int32u>(status_data->tactile_data_type))
      {
        // TACTILE DATA
        case TACTILE_SENSOR_TYPE_BIOTAC_INVALID:
          ROS_WARN("received invalid tactile type");
          break;

        case TACTILE_SENSOR_TYPE_BIOTAC_PDC:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.pdc = static_cast<int>(tactile_data->other_sensor_0);
          }
          else
          {
            // TODO(shadow_team): add some error stats
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.tac = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_TAC:
        //   this->all_tactile_data->at(id_sensor).biotac.tac =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]));
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_TDC:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.tdc = static_cast<int>(tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[0] = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_1:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[0] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_2:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[1] = static_cast<int>(tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[2] = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_3:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[2] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_4:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[3] = static_cast<int>(tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[4] = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_5:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[4] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_6:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[5] = static_cast<int>(tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[6] = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_7:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[6] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_8:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[7] = static_cast<int>(tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[8] = static_cast<int>(tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_9:
        //  this->all_tactile_data->at(id_sensor).biotac.electrodes[8] =
        //    static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //  break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_10:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[9] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[10] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_11:
        //    this->all_tactile_data->at(id_sensor).biotac.electrodes[10] =
        //      static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //    break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_12:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[11] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[12] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_13:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[12] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_14:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[13] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[14] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_15:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[14] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_16:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[15] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[16] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_17:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[16] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_18:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[17] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[18] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        // case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_19:
        //   this->all_tactile_data->at(id_sensor).biotac.electrodes[18] =
        //     static_cast<int>(static_cast<int16u>(status_data->tactile[id_sensor].word[2]) );
        //   break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_20:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[19] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[20] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_22:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[21] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          if (tactile_data->data_valid.other_sensor_1)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[22] = static_cast<int>(
              tactile_data->other_sensor_1);
          }
          break;

        case TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_24:
          if (tactile_data->data_valid.other_sensor_0)
          {
            this->all_tactile_data->at(id_sensor).biotac.electrodes[23] = static_cast<int>(
              tactile_data->other_sensor_0);
          }
          // if (tactile_data->data_valid.other_sensor_1)
          // {
          //   this->all_tactile_data->at(id_sensor).biotac.pdc = static_cast<int>(tactile_data->other_sensor_1);
          // }
          break;

          // COMMON DATA
        case TACTILE_SENSOR_TYPE_SAMPLE_FREQUENCY_HZ:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            this->all_tactile_data->at(id_sensor).biotac.sample_frequency =
                    static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[id_sensor].word[0]));
          }
          break;

        case TACTILE_SENSOR_TYPE_MANUFACTURER:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            this->all_tactile_data->at(id_sensor).biotac.manufacturer = this->sanitise_string(
              status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES);
          }
          break;

        case TACTILE_SENSOR_TYPE_SERIAL_NUMBER:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            this->all_tactile_data->at(id_sensor).biotac.serial_number = this->sanitise_string(
              status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES);
          }
          break;

        case TACTILE_SENSOR_TYPE_SOFTWARE_VERSION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            this->all_tactile_data->at(id_sensor).biotac.set_software_version(status_data->tactile[id_sensor].string);
          }
          break;

        case TACTILE_SENSOR_TYPE_PCB_VERSION:
          if (sr_math_utils::is_bit_mask_index_true(tactile_mask, id_sensor))
          {
            this->all_tactile_data->at(id_sensor).biotac.pcb_version = this->sanitise_string(
              status_data->tactile[id_sensor].string, TACTILE_DATA_LENGTH_BYTES);
          }
          break;

        default:
          break;
      }  // end switch
    }  // end for tactile

    if (this->sensor_updater->update_state == operation_mode::device_update_state::INITIALIZATION)
    {
      this->process_received_data_type(static_cast<int32u>(status_data->tactile_data_type));
      if (this->sensor_updater->initialization_configs_vector.size() == 0)
      {
        this->sensor_updater->update_state = operation_mode::device_update_state::OPERATION;
      }
    }
  }

  template<class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::publish()
  {
    // left empty, this is published from the controller publisher
  }  // end publish

  template <class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                                        diagnostic_updater::DiagnosticStatusWrapper &d)
  {
    for (unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
    {
      std::stringstream ss;
      std::string prefix = this->device_id_.empty() ? this->device_id_ : (this->device_id_ + " ");

      ss << prefix << "Tactile " << id_tact + 1;

      d.name = ss.str().c_str();
      d.summary(d.OK, "OK");
      d.clear();

      d.addf("Sample Frequency", "%d", this->all_tactile_data->at(id_tact).biotac.sample_frequency);
      d.addf("Manufacturer", "%s", this->all_tactile_data->at(id_tact).biotac.manufacturer.c_str());
      d.addf("Serial Number", "%s", this->all_tactile_data->at(id_tact).biotac.serial_number.c_str());

      d.addf("Software Version", "%s", this->all_tactile_data->at(id_tact).biotac.get_software_version().c_str());
      d.addf("PCB Version", "%s", this->all_tactile_data->at(id_tact).biotac.pcb_version.c_str());

      vec.push_back(d);
    }
  }

  template<class StatusType, class CommandType>
  std::vector<AllTactileData> *Biotac<StatusType, CommandType>::get_tactile_data()
  {
    return this->all_tactile_data.get();
  }

  template <class StatusType, class CommandType>
  void Biotac<StatusType, CommandType>::set_version_specific_details()
  {
    nb_electrodes_ = nb_electrodes_v1_;  // We consider biotac version one to be the default

    for (size_t i = 0; i < this->nb_tactiles; ++i)
    {
      // At least one of the fingers has a newer serial number, i.e. starting BTSP or BTTB
      if (this->all_tactile_data->at(i).biotac.serial_number.find("BTSP") != std::string::npos ||
          this->all_tactile_data->at(i).biotac.serial_number.find("BTTB") != std::string::npos)
      {
        nb_electrodes_ = nb_electrodes_v2_;
        break;
      }
    }

    if (nb_electrodes_ == nb_electrodes_v1_)  // If biotac version 1 remove polling for non-existing electrodes
    {
      for (int32u data = TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_20;
           data <= TACTILE_SENSOR_TYPE_BIOTAC_ELECTRODE_24; ++data)
      {
        std::vector<generic_updater::UpdateConfig>::iterator it =
          this->sensor_updater->important_update_configs_vector.begin();
        while (it != this->sensor_updater->important_update_configs_vector.end())
        {
          if (it->what_to_update == data)
          {
            it = this->sensor_updater->important_update_configs_vector.erase(it);
          }
          else
          {
            it++;
          }
        }
      }
    }

    for (unsigned int id_tact = 0; id_tact < this->nb_tactiles; ++id_tact)
    {
      this->all_tactile_data->at(id_tact).biotac.electrodes.resize(nb_electrodes_);
    }
  }

  // Only to ensure that the template class is compiled for the types we are interested in
  template
  class Biotac<ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND>;

  template
  class Biotac<ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0230_PALM_EDC_COMMAND>;

  template
  class Biotac<ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0240_PALM_EDC_COMMAND>;

  template
  class Biotac<ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0250_PALM_EDC_COMMAND>;

  template
  class Biotac<ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_STATUS, ETHERCAT_DATA_STRUCTURE_0300_PALM_EDC_COMMAND>;
}  // namespace tactiles

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
