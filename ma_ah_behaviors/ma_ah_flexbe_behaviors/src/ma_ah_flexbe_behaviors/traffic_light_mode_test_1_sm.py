#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.lane_control import ControlLaneStateTo
from ma_ah_flexbe_states.traffic_light import TrafficLightState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 02 2023
@author: sion jeon
'''
class traffic_light_mode_test_1SM(Behavior):
	'''
	enables to robot moves when traffic light "green"
	'''


	def __init__(self):
		super(traffic_light_mode_test_1SM, self).__init__()
		self.name = 'traffic_light_mode_test_1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:472, x:130 y:472
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:213 y:48
			OperatableStateMachine.add('TrafficLightMode',
										TrafficLightState(),
										transitions={'green_light': 'LaneControlMode', 'red_or_yellow_light': 'TrafficLightMode', 'no_signal': 'TrafficLightMode'},
										autonomy={'green_light': Autonomy.Off, 'red_or_yellow_light': Autonomy.Off, 'no_signal': Autonomy.Off})

			# x:435 y:199
			OperatableStateMachine.add('LaneControlMode',
										ControlLaneStateTo(),
										transitions={'proceed': 'LaneControlMode', 'left_lane': 'LaneControlMode'},
										autonomy={'proceed': Autonomy.Off, 'left_lane': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
