#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.find_lane_control_parking import ControlLaneFindParkingState
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
		target_theta = 1.53
		target_distance = 0.23
		# x:1084 y:577, x:130 y:350
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.stop = stop
		_state_machine.userdata.back = back
		_state_machine.userdata.middle = middle
		_state_machine.userdata.target_theta = target_theta
		_state_machine.userdata.target_distance = target_distance

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:442 y:63
			OperatableStateMachine.add('control_lane_find_parking',
										ControlLaneFindParkingState(),
										transitions={'left': 'find_parking_left', 'right': 'find_parking_right_2 find_parking_right_2', 'proceed': 'control_lane_find_parking'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off},
										remapping={'lane_info': 'right'})

			# x:303 y:341
			OperatableStateMachine.add('escape_parking_back',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back', 'done': 'escape_parking_left'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:544 y:335
			OperatableStateMachine.add('escape_parking_back_2',
										MovingControlState(),
										transitions={'procced': 'escape_parking_back_2', 'done': 'escape_parking_right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'back', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:305 y:420
			OperatableStateMachine.add('escape_parking_left',
										MovingControlState(),
										transitions={'procced': 'escape_parking_left', 'done': 'find_parking_go_3'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:559 y:418
			OperatableStateMachine.add('escape_parking_right',
										MovingControlState(),
										transitions={'procced': 'escape_parking_right', 'done': 'find_parking_go_3'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:319 y:245
			OperatableStateMachine.add('find_parking_go',
										MovingControlState(),
										transitions={'procced': 'find_parking_go', 'done': 'escape_parking_back'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:578 y:249
			OperatableStateMachine.add('find_parking_go_2',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_2', 'done': 'escape_parking_back_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:444 y:528
			OperatableStateMachine.add('find_parking_go_3',
										MovingControlState(),
										transitions={'procced': 'find_parking_go_3', 'done': 'finished'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:349 y:151
			OperatableStateMachine.add('find_parking_left',
										MovingControlState(),
										transitions={'procced': 'find_parking_left', 'done': 'find_parking_go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:576 y:151
			OperatableStateMachine.add('find_parking_right_2 find_parking_right_2',
										MovingControlState(),
										transitions={'procced': 'find_parking_right_2 find_parking_right_2', 'done': 'find_parking_go_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
