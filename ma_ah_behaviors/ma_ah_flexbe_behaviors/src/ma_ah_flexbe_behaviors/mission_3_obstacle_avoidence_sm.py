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
from ma_ah_flexbe_states.moving_control_state import MovingControlState
from ma_ah_flexbe_states.obstacle_control_lane_state import ObstacleControlLaneState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 02 2023
@author: ggh-png
'''
class mission3obstacleavoidenceSM(Behavior):
	'''
	mission 3 obstacle avoidence
	'''


	def __init__(self):
		super(mission3obstacleavoidenceSM, self).__init__()
		self.name = 'mission 3 obstacle avoidence'

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
		middle = "middle"
		target_theta = 1.6
		target_distance = 2
		scan_info = [85, 95]
		# x:670 y:88, x:145 y:311
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.target_theta = target_theta
		_state_machine.userdata.target_distance = target_distance
		_state_machine.userdata.middle = middle
		_state_machine.userdata.scan_info = scan_info

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:447 y:79
			OperatableStateMachine.add('obstacle_lane_control',
										ObstacleControlLaneState(),
										transitions={'mission_control': 'left'},
										autonomy={'mission_control': Autonomy.Off},
										remapping={'lane_info': 'right', 'scan_info': 'scan_info'})

			# x:538 y:621
			OperatableStateMachine.add('go_3',
										MovingControlState(),
										transitions={'procced': 'go_3', 'done': 'left_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:866 y:105
			OperatableStateMachine.add('lane_control_start',
										ControlLaneState(),
										transitions={'lane_control': 'lane_control_start', 'mission_control': 'obstacle_lane_control_3'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle'})

			# x:192 y:167
			OperatableStateMachine.add('left',
										MovingControlState(),
										transitions={'procced': 'left', 'done': 'go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:710 y:515
			OperatableStateMachine.add('left_2',
										MovingControlState(),
										transitions={'procced': 'left_2', 'done': 'obstacle_lane_control_3'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:102 y:443
			OperatableStateMachine.add('obstacle_lane_control_2',
										ObstacleControlLaneState(),
										transitions={'mission_control': 'right_2'},
										autonomy={'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left', 'scan_info': 'scan_info'})

			# x:723 y:383
			OperatableStateMachine.add('obstacle_lane_control_3',
										ObstacleControlLaneState(),
										transitions={'mission_control': 'finished'},
										autonomy={'mission_control': Autonomy.Off},
										remapping={'lane_info': 'left', 'scan_info': 'scan_info'})

			# x:259 y:340
			OperatableStateMachine.add('right',
										MovingControlState(),
										transitions={'procced': 'right', 'done': 'obstacle_lane_control_2'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:372 y:541
			OperatableStateMachine.add('right_2',
										MovingControlState(),
										transitions={'procced': 'right_2', 'done': 'go_3'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:213 y:253
			OperatableStateMachine.add('go',
										MovingControlState(),
										transitions={'procced': 'go', 'done': 'right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
