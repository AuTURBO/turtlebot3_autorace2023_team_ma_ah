#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.moving_control_state import MovingControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Oct 08 2023
@author: ggh-png
'''
class movingtestSM(Behavior):
	'''
	moving test
	'''


	def __init__(self):
		super(movingtestSM, self).__init__()
		self.name = 'moving test'

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
		back = "back"
		target_distance = 0.3
		target_theta = 1.57
		target_theta_1 = 3.14
		# x:718 y:329, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go
		_state_machine.userdata.back = back
		_state_machine.userdata.target_distance = target_distance
		_state_machine.userdata.target_theta = target_theta
		_state_machine.userdata.target_theta_1 = target_theta_1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:193 y:160
			OperatableStateMachine.add('left',
										MovingControlState(),
										transitions={'procced': 'left', 'done': 'go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:193 y:299
			OperatableStateMachine.add('right',
										MovingControlState(),
										transitions={'procced': 'right', 'done': 'finished'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})

			# x:193 y:237
			OperatableStateMachine.add('go',
										MovingControlState(),
										transitions={'procced': 'go', 'done': 'right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go', 'target_distance': 'target_distance', 'target_theta': 'target_theta'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
