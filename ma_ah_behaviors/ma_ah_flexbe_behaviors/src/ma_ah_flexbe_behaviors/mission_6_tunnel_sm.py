#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from ma_ah_flexbe_states.lane_control import ControlLaneState
from ma_ah_flexbe_states.move_base import MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 06 2023
@author: ggh-png
'''
class mission6tunnelSM(Behavior):
	'''
	mission 6 tunnel
	'''


	def __init__(self):
		super(mission6tunnelSM, self).__init__()
		self.name = 'mission 6 tunnel'

		# parameters of this behavior
		self.add_parameter('wait_time', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		initial_pose = [-2.5, 2.2, -1.57]
		middle = "middle"
		waypoint = [0.2, -1.75, 0]
		# x:30 y:638, x:130 y:638
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.initial_pose = initial_pose
		_state_machine.userdata.middle = middle
		_state_machine.userdata.waypoint = waypoint

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:191 y:101
			OperatableStateMachine.add('wait_state',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'goal_move_base'},
										autonomy={'done': Autonomy.Off})

			# x:646 y:376
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:491 y:167
			OperatableStateMachine.add('goal_move_base',
										MoveBaseState(),
										transitions={'arrived': 'lane_control', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
