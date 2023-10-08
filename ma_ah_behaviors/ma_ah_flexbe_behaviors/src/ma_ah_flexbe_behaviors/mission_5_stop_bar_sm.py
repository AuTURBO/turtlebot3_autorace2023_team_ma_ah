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
from ma_ah_flexbe_states.left_state import LeftSignState
from ma_ah_flexbe_states.stop_bar_state import StopBarState
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
		right = "right"
		middle = "middle"
		# x:30 y:638, x:130 y:638
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.middle = middle

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:303 y:44
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'left sign state'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:272 y:415
			OperatableStateMachine.add('lane_control_2',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_2', 'mission_control': 'left sign state'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left'})

			# x:173 y:251
			OperatableStateMachine.add('left sign state',
										LeftSignState(),
										transitions={'turn_left': 'lane_control_2', 'proceed': 'lane_control', 'stop': 'stop bar'},
										autonomy={'turn_left': Autonomy.Off, 'proceed': Autonomy.Off, 'stop': Autonomy.Off})

			# x:602 y:196
			OperatableStateMachine.add('stop bar',
										StopBarState(),
										transitions={'proceed': 'lane_control', 'done': 'finished'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
