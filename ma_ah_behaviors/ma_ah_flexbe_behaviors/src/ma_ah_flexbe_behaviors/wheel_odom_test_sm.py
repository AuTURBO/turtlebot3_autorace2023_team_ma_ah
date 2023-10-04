#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.control_mode_state import ControlModeState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 03 2023
@author: sion
'''
class wheelodomtestSM(Behavior):
	'''
	enables to robot move using wheel odom
	'''


	def __init__(self):
		super(wheelodomtestSM, self).__init__()
		self.name = 'wheel odom test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:182 y:93
			OperatableStateMachine.add('ControlModeState',
										ControlModeState(),
										transitions={'lane_control': 'ControlModeState', 'mission_control': 'ControlModeState'},
										autonomy={'lane_control': Autonomy.Off, 'mission_control': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
