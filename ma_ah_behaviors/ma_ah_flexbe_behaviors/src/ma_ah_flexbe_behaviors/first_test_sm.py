#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_behaviors.mission_1_traffic_light_sm import mission1trafficlightSM
from ma_ah_flexbe_behaviors.mission_2_cross_line__sm import mission2crosslineSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 09 2023
@author: sion jeon
'''
class first_testSM(Behavior):
	'''
	hi_hellolo
	'''


	def __init__(self):
		super(first_testSM, self).__init__()
		self.name = 'first_test'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(mission1trafficlightSM, 'mission 1 traffic light')
		self.add_behavior(mission2crosslineSM, 'mission 2 cross line ')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		left = "left"
		right = "right"
		middle = "middle"
		# x:30 y:365, x:670 y:106
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = left
		_state_machine.userdata.right = right
		_state_machine.userdata.middle = middle

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:145 y:69
			OperatableStateMachine.add('mission 1 traffic light',
										self.use_behavior(mission1trafficlightSM, 'mission 1 traffic light'),
										transitions={'finished': 'mission 2 cross line ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:358 y:210
			OperatableStateMachine.add('mission 2 cross line ',
										self.use_behavior(mission2crosslineSM, 'mission 2 cross line '),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
