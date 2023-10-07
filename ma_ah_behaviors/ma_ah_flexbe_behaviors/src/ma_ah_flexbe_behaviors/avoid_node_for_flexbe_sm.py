#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.avoid_for_flexbe_state import ObstacleDetectorState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 07 2023
@author: sion jeon
'''
class avoid_node_for_flexbeSM(Behavior):
	'''
	change avoid node for flexbe
	'''


	def __init__(self):
		super(avoid_node_for_flexbeSM, self).__init__()
		self.name = 'avoid_node_for_flexbe'

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
			# x:95 y:59
			OperatableStateMachine.add('ObstacleDetectorMode',
										ObstacleDetectorState(),
										transitions={'obstacle': 'ObstacleDetectorMode', 'done': 'ObstacleDetectorMode'},
										autonomy={'obstacle': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
