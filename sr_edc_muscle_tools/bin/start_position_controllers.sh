#!/bin/bash
# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# We want only the position controllers running, so stop all running
for name in $(rosrun controller_manager controller_manager list | grep running | cut -d ' ' -f 1); do
    rosrun controller_manager controller_manager stop $name
done
# Start position controllers
for muscle in ffj0 ffj3 ffj4 rfj0 rfj3 rfj4 thj2 thj3 thj4 thj5 wrj1 wrj2; do
    rosrun controller_manager controller_manager spawn "sh_${muscle}_muscle_position_controller"
done
