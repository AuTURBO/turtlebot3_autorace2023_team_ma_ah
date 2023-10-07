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
from ma_ah_flexbe_behaviors.mission_3_obstacle_avoidence_sm import mission3obstacleavoidenceSM
from ma_ah_flexbe_behaviors.mission_4_parking_sm import mission4parkingSM
from ma_ah_flexbe_behaviors.mission_5_stop_bar_sm import mission5stopbarSM
from ma_ah_flexbe_behaviors.mission_6_tunnel_sm import mission6tunnelSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 03 2023
@author: ggh-png
'''
class maahautoracefsmSM(Behavior):
	'''
	ma ah autorace fsm
	'''


	def __init__(self):
		super(maahautoracefsmSM, self).__init__()
		self.name = 'ma ah autorace fsm'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(mission1trafficlightSM, 'mission 1 traffic light')
		self.add_behavior(mission2crosslineSM, 'mission 2 cross line ')
		self.add_behavior(mission3obstacleavoidenceSM, 'mission 3 obstacle avoidence')
		self.add_behavior(mission4parkingSM, 'mission 4 parking')
		self.add_behavior(mission5stopbarSM, 'mission 5 stop bar')
		self.add_behavior(mission6tunnelSM, 'mission 6 tunnel')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		right = "right"
		# x:30 y:365, x:165 y:366
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.right = right

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:118 y:39
			OperatableStateMachine.add('mission 1 traffic light',
										self.use_behavior(mission1trafficlightSM, 'mission 1 traffic light'),
										transitions={'finished': 'mission 2 cross line ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:212 y:137
			OperatableStateMachine.add('mission 2 cross line ',
										self.use_behavior(mission2crosslineSM, 'mission 2 cross line '),
										transitions={'finished': 'mission 3 obstacle avoidence', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:427 y:184
			OperatableStateMachine.add('mission 3 obstacle avoidence',
										self.use_behavior(mission3obstacleavoidenceSM, 'mission 3 obstacle avoidence'),
										transitions={'finished': 'mission 4 parking', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:557 y:360
			OperatableStateMachine.add('mission 4 parking',
										self.use_behavior(mission4parkingSM, 'mission 4 parking'),
										transitions={'finished': 'mission 5 stop bar', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:668 y:503
			OperatableStateMachine.add('mission 5 stop bar',
										self.use_behavior(mission5stopbarSM, 'mission 5 stop bar'),
										transitions={'finished': 'mission 6 tunnel', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:828 y:658
			OperatableStateMachine.add('mission 6 tunnel',
										self.use_behavior(mission6tunnelSM, 'mission 6 tunnel'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
