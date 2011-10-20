/**
 * @file   generic_tactiles.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Th Oct 20 10:06:14 2011
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

#ifndef GENERIC_TACTILES_HPP_
#define GENERIC_TACTILES_HPP_

namespace tactiles
{
  class GenericTactiles
  {
  public:
    GenericTactiles();
    ~GenericTactiles() {};

      virtual update(void) = 0;

      virtual build_command(void) = 0;

      /// Number of tactile sensors (TODO: should probably be defined in the protocol)
      static const unsigned int nb_tactiles;

  };//end class
}//end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* GENERIC_TACTILES_HPP_ */
