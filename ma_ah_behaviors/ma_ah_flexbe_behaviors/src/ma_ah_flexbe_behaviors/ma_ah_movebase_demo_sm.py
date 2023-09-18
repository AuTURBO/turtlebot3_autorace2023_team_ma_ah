#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from ma_ah_flexbe_states.move_base_state import MoveBaseState as ma_ah_flexbe_states__MoveBaseState
from ma_ah_flexbe_states.traffic_sign import TrafficSign
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Sep 18 2023
@author: ggh-png
'''
class maahmovebasedemoSM(Behavior):
	'''
	turtlebot movebase using flexbe
	'''


	def __init__(self):
		super(maahmovebasedemoSM, self).__init__()
		self.name = 'ma ah movebase demo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		init_waypoint = [-2.0, -2.0, 1]
		arrive = "end waypoint"
		failed = "failed waypoint"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.waypoint = init_waypoint

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:470 y:52
			OperatableStateMachine.add('line_control',
										TrafficSign(),
										transitions={'continue': 'line_control', 'obstacle': 'waypoint', 'traffic_light': 'line_control', 'parking': 'line_control', 'cross': 'line_control'},
										autonomy={'continue': Autonomy.Off, 'obstacle': Autonomy.Off, 'traffic_light': Autonomy.Off, 'parking': Autonomy.Off, 'cross': Autonomy.Off})

			# x:564 y:400
			OperatableStateMachine.add('not end',
										LogState(text=failed, severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:201 y:141
			OperatableStateMachine.add('waypoint',
										ma_ah_flexbe_states__MoveBaseState(),
										transitions={'arrived': 'end', 'failed': 'not end'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:302 y:303
			OperatableStateMachine.add('end',
										LogState(text=arrive, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
