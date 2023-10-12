#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.find_lane_control_parking import ControlLaneFindParkingState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 13 2023
@author: ggh
'''
class findparkingtestSM(Behavior):
	'''
	find parking test
	'''


	def __init__(self):
		super(findparkingtestSM, self).__init__()
		self.name = 'find parking test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		left = "left"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:308 y:101
			OperatableStateMachine.add('find_parking',
										ControlLaneFindParkingState(),
										transitions={'left': 'finished', 'right': 'failed', 'proceed': 'find_parking'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off},
										remapping={'lane_info': 'left'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
