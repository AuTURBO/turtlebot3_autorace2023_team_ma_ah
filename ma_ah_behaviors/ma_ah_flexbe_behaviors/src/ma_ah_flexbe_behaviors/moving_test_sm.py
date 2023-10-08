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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.go = go

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:28
			OperatableStateMachine.add('left',
										MovingControlState(),
										transitions={'procced': 'left', 'done': 'go'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'left'})

			# x:60 y:167
			OperatableStateMachine.add('right',
										MovingControlState(),
										transitions={'procced': 'right', 'done': 'finished'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'right'})

			# x:224 y:120
			OperatableStateMachine.add('go',
										MovingControlState(),
										transitions={'procced': 'go', 'done': 'right'},
										autonomy={'procced': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'moving_info': 'go'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
