#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.lane_control import ControlLaneState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 06 2023
@author: ggh-png
'''
class mission5stopbarSM(Behavior):
	'''
	mission 5 stop bar
	'''


	def __init__(self):
		super(mission5stopbarSM, self).__init__()
		self.name = 'mission 5 stop bar'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		left = "left"
		stop = "stop"
		middle = "middle"
		pid_info = [0.8, 0.0, 0.3]
		vel_info = 0.05
		right = "right"
		pid_pid_2 = [0.8, 0.0, 0.3]
		# x:30 y:638, x:130 y:638
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.stop = stop
		_state_machine.userdata.pid_info = pid_info
		_state_machine.userdata.vel_info = vel_info
		_state_machine.userdata.right = right
		_state_machine.userdata.middle = middle

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:275 y:290
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'right', 'pid_info': 'pid_info', 'vel_info': 'vel_info'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
