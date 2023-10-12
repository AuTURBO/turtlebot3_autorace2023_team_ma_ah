#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.find_lane_control_parking import FindParkingStateObs
from ma_ah_flexbe_states.obstacle_control_lane_state import ObstacleControlLaneState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 12 2023
@author: ggh
'''
class obdtacleSM(Behavior):
	'''
	obdtacle
	'''


	def __init__(self):
		super(obdtacleSM, self).__init__()
		self.name = 'obdtacle'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		middle = "middle"
		scan_info = [85, 95]
		right = "right"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.middle = middle
		_state_machine.userdata.scan_info = scan_info
		_state_machine.userdata.right = right

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:215 y:68
			OperatableStateMachine.add('dd',
										FindParkingStateObs(),
										transitions={'left': 'finished', 'right': 'lanbe_control', 'proceed': 'dd'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off, 'proceed': Autonomy.Off},
										remapping={'lane_info': 'right'})

			# x:493 y:167
			OperatableStateMachine.add('lanbe_control',
										ObstacleControlLaneState(),
										transitions={'mission_control': 'finished'},
										autonomy={'mission_control': Autonomy.Off},
										remapping={'lane_info': 'middle', 'scan_info': 'scan_info'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
