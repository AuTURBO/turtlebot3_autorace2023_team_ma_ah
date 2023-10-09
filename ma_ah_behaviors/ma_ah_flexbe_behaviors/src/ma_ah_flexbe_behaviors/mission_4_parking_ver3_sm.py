#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.find_parking import FindParkingState
from ma_ah_flexbe_states.lane_control import ControlLaneState
from ma_ah_flexbe_states.moving_control_state import MovingControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 05 2023
@author: sion jeon
'''
class mission4parking_ver3SM(Behavior):
	'''
	mission 4 parking
	'''


	def __init__(self):
		super(mission4parking_ver3SM, self).__init__()
		self.name = 'mission 4 parking_ver3'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		left = "left"
		right = "right"
		go = "go"
		stop = "stop"
		back = "back"
		middle = "middle"
		target_distance = 0.33
		target_theta = 1.57
		escape_distance = 0.95
		# x:1566 y:207, x:130 y:333
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.stop = stop
		_state_machine.userdata.back = back
		_state_machine.userdata.middle = middle
		_state_machine.userdata.target_distance = target_distance
		_state_machine.userdata.target_theta = target_theta
		_state_machine.userdata.escape_distance = escape_distance

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:402 y:55
			OperatableStateMachine.add('find_parking',
										FindParkingState(),
										transitions={'left': 'find_parking_left', 'right': 'find_parking_right', 'proceed': 'find_parking'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off})

			# x:850 y:766
			OperatableStateMachine.add('escape_lane_control_2',
										ControlLaneState(),
										transitions={'lane_control': 'escape_lane_control_2', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left'})

			# x:659 y:775
			OperatableStateMachine.add('escape_left',
										MovingControlState(),
										transitions={'procced': 'escape_left', 'done': 'escape_lane_control_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:277 y:535
			OperatableStateMachine.add('escape_parking_back',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back', 'done': 'escape_parking_left'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:556 y:554
			OperatableStateMachine.add('escape_parking_back_2',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back_2', 'done': 'escape_parking_right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:302 y:657
			OperatableStateMachine.add('escape_parking_left',
										MovingControlState(),
										transitions={'procced': 'escape_parking_left', 'done': 'escape_go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:548 y:664
			OperatableStateMachine.add('escape_parking_right',
										MovingControlState(),
										transitions={'procced': 'escape_parking_right', 'done': 'escape_go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:280 y:436
			OperatableStateMachine.add('find_parking_go',
										MovingControlState(),
										transitions={'procced': 'find_parking_go', 'done': 'escape_parking_back'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:533 y:435
			OperatableStateMachine.add('find_parking_go_2',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_2', 'done': 'escape_parking_back_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:265 y:311
			OperatableStateMachine.add('find_parking_left',
										MovingControlState(),
										transitions={'procced': 'find_parking_left', 'done': 'find_parking_go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:524 y:319
			OperatableStateMachine.add('find_parking_right',
										MovingControlState(),
										transitions={'procced': 'find_parking_right', 'done': 'find_parking_go_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:430 y:755
			OperatableStateMachine.add('escape_go',
										MovingControlState(),
										transitions={'procced': 'escape_go', 'done': 'escape_left'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'escape_distance', 'target_theta': 'target_theta'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
