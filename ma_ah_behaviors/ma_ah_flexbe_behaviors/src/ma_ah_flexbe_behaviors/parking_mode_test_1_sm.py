#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.parking_state import ParkingState
from ma_ah_flexbe_states.traffic_sign import TrafficSign
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 02 2023
@author: sion jeon
'''
class parking_mode_test_1SM(Behavior):
	'''
	enables to robot find and move parking area
	'''


	def __init__(self):
		super(parking_mode_test_1SM, self).__init__()
		self.name = 'parking_mode_test_1'

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
			# x:187 y:90
			OperatableStateMachine.add('TrafficSignMode',
										TrafficSign(),
										transitions={'intersection_sign': 'TrafficSignMode', 'obstacle_detection': 'TrafficSignMode', 'parking_detection': 'ParkingMode', 'stop_bar_status': 'TrafficSignMode', 'tunnel_info': 'TrafficSignMode'},
										autonomy={'intersection_sign': Autonomy.Off, 'obstacle_detection': Autonomy.Off, 'parking_detection': Autonomy.Off, 'stop_bar_status': Autonomy.Off, 'tunnel_info': Autonomy.Off})

			# x:414 y:211
			OperatableStateMachine.add('ParkingMode',
										ParkingState(),
										transitions={'parking_proceed': 'ParkingMode', 'done': 'ParkingMode'},
										autonomy={'parking_proceed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
