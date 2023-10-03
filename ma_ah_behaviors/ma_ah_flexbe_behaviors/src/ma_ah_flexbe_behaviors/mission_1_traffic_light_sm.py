#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.lane_control_2 import ControlLaneStateTo
from ma_ah_flexbe_states.traffic_light import TrafficLightState
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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:189 y:76
			OperatableStateMachine.add('traffic_lane_control',
										ControlLaneStateTo(),
										transitions={'lane_control': 'traffic_lane_control', 'mission_control': 'traffic_light'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off})

			# x:430 y:139
			OperatableStateMachine.add('traffic_light',
										TrafficLightState(),
										transitions={'green_light': 'finished', 'red_or_yellow_light': 'traffic_light', 'no_signal': 'traffic_light'},
										autonomy={'green_light': Autonomy.Off, 'red_or_yellow_light': Autonomy.Off, 'no_signal': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
