#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.find_lane_control_parking import FindParkingStateObs
from ma_ah_flexbe_states.lane_control import ControlLaneState
from ma_ah_flexbe_states.moving_control_state import MovingControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 05 2023
@author: ggh-png
'''
class mission4parkingSM(Behavior):
	'''
	mission 4 parking
	'''


	def __init__(self):
		super(mission4parkingSM, self).__init__()
		self.name = 'mission 4 parking'

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
		target_theta = 1.6
		target_distance = 0.3
		escape_go = 0.45
		two_left = "two_left"
		pid_info = [0.8, 0.0, 0.3]
		vel_info = 0.05
		pid_info_2 = [0.35, 0.0, 0.1]
		# x:1184 y:252, x:130 y:350
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.stop = stop
		_state_machine.userdata.back = back
		_state_machine.userdata.middle = middle
		_state_machine.userdata.target_theta = target_theta
		_state_machine.userdata.target_distance = target_distance
		_state_machine.userdata.escape_go = escape_go
		_state_machine.userdata.two_left = two_left
		_state_machine.userdata.pid_info = pid_info
		_state_machine.userdata.vel_info = vel_info
		_state_machine.userdata.pid_info_2 = pid_info_2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:394 y:194
			OperatableStateMachine.add('find_parking_lane_control',
										FindParkingStateObs(),
										transitions={'left': 'find_parking_go_3', 'right': 'find_parking_go_4', 'proceed': 'find_parking_lane_control'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off},
										remapping={'lane_info': 'two_left', 'pid_info': 'pid_info'})

			# x:277 y:535
			OperatableStateMachine.add('escape_parking_back',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back', 'done': 'escape_parking_left'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:496 y:525
			OperatableStateMachine.add('escape_parking_back_2',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back_2', 'done': 'escape_parking_right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:504 y:626
			OperatableStateMachine.add('escape_parking_left',
										MovingControlState(),
										transitions={'procced': 'escape_parking_left', 'done': 'find_parking_go_2_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:277 y:627
			OperatableStateMachine.add('escape_parking_right',
										MovingControlState(),
										transitions={'procced': 'escape_parking_right', 'done': 'find_parking_go_2_2'},
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

			# x:692 y:676
			OperatableStateMachine.add('find_parking_go_2_2',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_2_2', 'done': 'escape_lane_control'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'escape_go', 'target_theta': 'target_theta'})

			# x:159 y:251
			OperatableStateMachine.add('find_parking_go_3',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_3', 'done': 'find_parking_left'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'escape_go', 'target_theta': 'target_theta'})

			# x:655 y:225
			OperatableStateMachine.add('find_parking_go_4',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_4', 'done': 'find_parking_right_2 find_parking_right_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'escape_go', 'target_theta': 'target_theta'})

			# x:284 y:328
			OperatableStateMachine.add('find_parking_left',
										MovingControlState(),
										transitions={'procced': 'find_parking_left', 'done': 'find_parking_go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:524 y:319
			OperatableStateMachine.add('find_parking_right_2 find_parking_right_2',
										MovingControlState(),
										transitions={'procced': 'find_parking_right_2 find_parking_right_2', 'done': 'find_parking_go_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:850 y:236
			OperatableStateMachine.add('escape_lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'escape_lane_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left', 'pid_info': 'pid_info_2', 'vel_info': 'vel_info'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
