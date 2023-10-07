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
		# x:30 y:365, x:130 y:350
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.stop = stop
		_state_machine.userdata.back = back
		_state_machine.userdata.middle = middle

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:142 y:62
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'lane_control_2'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left'})

			# x:406 y:924
			OperatableStateMachine.add('escape_lane_control_2',
										ControlLaneState(),
										transitions={'lane_control': 'escape_lane_control_2', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left'})

			# x:277 y:535
			OperatableStateMachine.add('escape_parking_back',
										MovingControlState(),
										transitions={'success': 'escape_parking_right', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'back'})

			# x:496 y:525
			OperatableStateMachine.add('escape_parking_back_2',
										MovingControlState(),
										transitions={'success': 'escape_parking_left', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'back'})

			# x:504 y:626
			OperatableStateMachine.add('escape_parking_left',
										MovingControlState(),
										transitions={'success': 'escape_lane_control', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'right'})

			# x:277 y:627
			OperatableStateMachine.add('escape_parking_right',
										MovingControlState(),
										transitions={'success': 'escape_lane_control', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'right'})

			# x:406 y:210
			OperatableStateMachine.add('find_parking',
										FindParkingState(),
										transitions={'left': 'find_parking_left', 'right': 'find_parking_right_2 find_parking_right_2', 'proceed': 'find_parking'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off})

			# x:280 y:436
			OperatableStateMachine.add('find_parking_go',
										MovingControlState(),
										transitions={'success': 'escape_parking_back', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'go'})

			# x:533 y:435
			OperatableStateMachine.add('find_parking_go_2',
										MovingControlState(),
										transitions={'success': 'escape_parking_back_2', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'go'})

			# x:284 y:328
			OperatableStateMachine.add('find_parking_left',
										MovingControlState(),
										transitions={'success': 'find_parking_go', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'left'})

			# x:524 y:319
			OperatableStateMachine.add('find_parking_right_2 find_parking_right_2',
										MovingControlState(),
										transitions={'success': 'find_parking_go_2', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'moving_info': 'right'})

			# x:400 y:114
			OperatableStateMachine.add('lane_control_2',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_2', 'mission_control': 'find_parking'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:402 y:788
			OperatableStateMachine.add('escape_lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'escape_lane_control', 'mission_control': 'escape_lane_control_2'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
