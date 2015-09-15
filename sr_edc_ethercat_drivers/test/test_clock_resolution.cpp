/**
 * @file   test_clock_resolution.cpp
 * @author Hugo Elias <hugo@shadowrobot.com>, Ugo Cupcic <ugo@shadowrobot.com>,
 *         Toni Oliver <toni@shadowrobot.com>, contact <software@shadowrobot.com>
 *
 * Copyright 2013 Shadow Robot Company Ltd.
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
 * @brief
 *
 *
 */

#include <time.h>
#include <stdio.h>

int main(int argc, char *argv[])
{
  struct timespec nano_time;
  clock_getres(CLOCK_REALTIME, &nano_time);
  printf("clock resolution = %ld secs and %ld nsecs\n",
         nano_time.tv_sec, nano_time.tv_nsec);
}
