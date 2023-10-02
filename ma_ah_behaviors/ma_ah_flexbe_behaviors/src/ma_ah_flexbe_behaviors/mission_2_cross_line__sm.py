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
from ma_ah_flexbe_states.lane_control_2 import ControlLaneStateTo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 03 2023
@author: ggh-png
'''
class mission2crosslineSM(Behavior):
	'''
	ma ah mission 2 cross line 
	'''


	def __init__(self):
		super(mission2crosslineSM, self).__init__()
		self.name = 'mission 2 cross line '

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:348 y:420, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:170 y:73
			OperatableStateMachine.add('cross_lane_control',
										ControlLaneStateTo(),
										transitions={'lane_control': 'cross_lane_control', 'mission_control': 'left_or_right_lane_control'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off})

			# x:84 y:249
			OperatableStateMachine.add('left_line_control',
										ControlLaneStateTo(),
										transitions={'lane_control': 'left_line_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off})

			# x:342 y:135
			OperatableStateMachine.add('left_or_right_lane_control',
										IntersectionState(),
										transitions={'turn_left': 'left_line_control', 'turn_right': 'right_line_control'},
										autonomy={'turn_left': Autonomy.Off, 'turn_right': Autonomy.Off})

			# x:456 y:242
			OperatableStateMachine.add('right_line_control',
										ControlLaneStateTo(),
										transitions={'lane_control': 'right_line_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
