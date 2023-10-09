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
from ma_ah_flexbe_states.lane_control import ControlLaneState
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
			# x:166 y:64
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'IntersectionMode'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:236 y:202
			OperatableStateMachine.add('lane_control_left',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_left', 'mission_control': 'lane_control_right_2'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left'})

			# x:504 y:204
			OperatableStateMachine.add('lane_control_right',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_right', 'mission_control': 'lane_control_right_2'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'right'})

			# x:394 y:343
			OperatableStateMachine.add('lane_control_right_2',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_right_2', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:435 y:65
			OperatableStateMachine.add('IntersectionMode',
										IntersectionState(),
										transitions={'turn_left': 'lane_control_left', 'turn_right': 'lane_control_right', 'proceed': 'lane_control'},
										autonomy={'turn_left': Autonomy.Off, 'turn_right': Autonomy.Off, 'proceed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
