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
from ma_ah_flexbe_states.traffic_sign_size_state import TrafficLightState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 02 2023
@author: ggh-png
'''
class mission1trafficlightSM(Behavior):
	'''
	ma ah mission 1 traffic light
	'''


	def __init__(self):
		super(mission1trafficlightSM, self).__init__()
		self.name = 'mission 1 traffic light'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		middle = "middle"
		left = "left"
		right = "right"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.middle = middle
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:125 y:60
			OperatableStateMachine.add('traffic_light_state',
										TrafficLightState(),
										transitions={'proceed': 'traffic_light_state', 'done': 'control_lane'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:292 y:183
			OperatableStateMachine.add('control_lane',
										ControlLaneState(),
										transitions={'lane_control': 'control_lane', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
