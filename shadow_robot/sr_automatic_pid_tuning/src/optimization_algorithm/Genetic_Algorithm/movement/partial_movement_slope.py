# -*- coding: utf-8 -*-
##
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#By: Emilie JEAN-BAPTISTE
##Date:24 Juin 2011

import numpy as np
from partial_movements import Partial_Movements
import roslib; roslib.load_manifest('sr_hand')
import rospy
from sr_hand.msg import joint,sendupdate,contrlr,joints_data

class Partial_Movement_Slope(Partial_Movements):
    def __init__(self,joint_name):
        Partial_Movements.__init__(self,joint_name)
        self.number_steps=230
        self.movement_name="Slope"
        self.ps=[]
        self.sequences=[]
        self.slope_incremental=0.0925
        return
        
    def compute_data(self,itr):
	"""
	Will compute the chosen movement
	@return: targets for sendupdate
	"""
	ptn=self.slope_incremental*itr
	self.sequences.append(ptn)
	if itr>=1:
	    if itr%2==0:
		self.ps.append("add")
		itr=len(self.ps)
	    else:
		itr=len(self.ps)
		
	
	return self.sequences[itr]

