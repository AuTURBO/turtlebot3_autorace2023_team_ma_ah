#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.intersection_state import IntersectionState
from ma_ah_flexbe_states.parking_state import ParkingState
from ma_ah_flexbe_states.traffic_sign import TrafficSign
from ma_ah_flexbe_states.traffic_sign_size_state import TrafficLightState
from ma_ah_flexbe_states.tunnel_state import TunnelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Oct 08 2023
@author: sion jeon
'''
class traffic_sign_change_testSM(Behavior):
	'''
	hi hello
	'''


	def __init__(self):
		super(traffic_sign_change_testSM, self).__init__()
		self.name = 'traffic_sign_change_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		left = "left"
		right = "right"
		middle = "middle"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.middle = middle

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:183 y:110
			OperatableStateMachine.add('TrafficSignMode',
										TrafficSign(),
										transitions={'intersection_sign': 'intersection', 'obstacle_detection': 'fake_obstacle', 'parking_detection': 'parking', 'stop_bar_status': 'fake_stop', 'tunnel_info': 'tunnel'},
										autonomy={'intersection_sign': Autonomy.Off, 'obstacle_detection': Autonomy.Off, 'parking_detection': Autonomy.Off, 'stop_bar_status': Autonomy.Off, 'tunnel_info': Autonomy.Off})

			# x:386 y:542
			OperatableStateMachine.add('fake_obstacle',
										TrafficLightState(),
										transitions={'proceed': 'fake_obstacle', 'done': 'TrafficSignMode'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:607 y:512
			OperatableStateMachine.add('fake_stop',
										TrafficLightState(),
										transitions={'proceed': 'fake_stop', 'done': 'TrafficSignMode'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:747 y:104
			OperatableStateMachine.add('intersection',
										IntersectionState(),
										transitions={'turn_left': 'TrafficSignMode', 'turn_right': 'TrafficSignMode', 'proceed': 'intersection'},
										autonomy={'turn_left': Autonomy.Off, 'turn_right': Autonomy.Off, 'proceed': Autonomy.Off})

			# x:690 y:225
			OperatableStateMachine.add('parking',
										ParkingState(),
										transitions={'parking_proceed': 'parking', 'done': 'TrafficSignMode'},
										autonomy={'parking_proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:675 y:342
			OperatableStateMachine.add('tunnel',
										TunnelState(),
										transitions={'proceed': 'tunnel', 'done': 'TrafficSignMode'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
