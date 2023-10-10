#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.c import ClearCostmapsState
from ma_ah_flexbe_states.lane_control import ControlLaneState
from ma_ah_flexbe_states.move_base import MoveBaseState
from ma_ah_flexbe_states.set_initial_pose_state import SetInitialPoseState
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
		initial_pose = [-2.38, 2.17, -1.57]
		middle = "middle"
		waypoint = [-0.7, 0.1, -0.003, 0.99]
		# x:30 y:638, x:130 y:638
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.initial_pose = initial_pose
		_state_machine.userdata.middle = middle
		_state_machine.userdata.waypoint = waypoint

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:465 y:32
			OperatableStateMachine.add('clear_costmaps',
										ClearCostmapsState(),
										transitions={'done': 'set_init', 'failed': 'clear_costmaps'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:251
			OperatableStateMachine.add('goal_move_base',
										MoveBaseState(),
										transitions={'arrived': 'lane_control', 'failed': 'goal_move_base'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:646 y:376
			OperatableStateMachine.add('lane_control',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control', 'mission_control': 'finished'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:726 y:115
			OperatableStateMachine.add('set_init',
										SetInitialPoseState(),
										transitions={'succeeded': 'goal_move_base'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'initial_pose': 'initial_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
