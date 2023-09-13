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
from flexbe_states.wait_state import WaitState
from ma_ah_flexbe_behaviors.ma_ah_test_sm import maahtestSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Sep 13 2023
@author: ggh
'''
class helloworldSM(Behavior):
	'''
	hello world demo
	'''


	def __init__(self):
		super(helloworldSM, self).__init__()
		self.name = 'hello world'

		# parameters of this behavior
		self.add_parameter('waiting_time', 1)

		# references to used behaviors
		self.add_behavior(maahtestSM, 'ma ah test')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hello = "hello world!!!"
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('init_wait',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'print_log'},
										autonomy={'done': Autonomy.Off})

			# x:146 y:251
			OperatableStateMachine.add('ma ah test',
										self.use_behavior(maahtestSM, 'ma ah test'),
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:279 y:142
			OperatableStateMachine.add('print_log',
										LogState(text=hello, severity=Logger.REPORT_HINT),
										transitions={'done': 'ma ah test'},
										autonomy={'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
