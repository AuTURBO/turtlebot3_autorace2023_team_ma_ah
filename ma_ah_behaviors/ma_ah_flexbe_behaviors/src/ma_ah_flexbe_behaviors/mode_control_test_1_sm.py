#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ma_ah_flexbe_states.intersection_state import IntersectionState
from ma_ah_flexbe_states.lane_control import ControlLaneState
from ma_ah_flexbe_states.lane_control_left import LeftControlLaneState
from ma_ah_flexbe_states.obstacle_detector_state import ObstacleDetectorState
from ma_ah_flexbe_states.parking_state import ParkingState
from ma_ah_flexbe_states.stop_bar_state import StopBarState
from ma_ah_flexbe_states.traffic_light import TrafficLightState
from ma_ah_flexbe_states.traffic_sign import TrafficSign
from ma_ah_flexbe_states.tunnel_state import TunnelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Sep 30 2023
@author: sion jeon
'''
class mode_control_test_1SM(Behavior):
	'''
	enables to robot control mode
	'''


	def __init__(self):
		super(mode_control_test_1SM, self).__init__()
		self.name = 'mode_control_test_1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:22 y:538, x:104 y:535
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:119 y:65
			OperatableStateMachine.add('TrafficLightState',
										TrafficLightState(),
										transitions={'green_light': 'LaneControlMode', 'red_or_yellow_light': 'TrafficLightState', 'no_signal': 'TrafficLightState'},
										autonomy={'green_light': Autonomy.Off, 'red_or_yellow_light': Autonomy.Off, 'no_signal': Autonomy.Off})

			# x:448 y:139
			OperatableStateMachine.add('LaneControlMode',
										ControlLaneState(),
										transitions={'proceed': 'LaneControlMode', 'traffic_sign': 'TrafficSignMode'},
										autonomy={'proceed': Autonomy.Off, 'traffic_sign': Autonomy.Off})

			# x:866 y:56
			OperatableStateMachine.add('LaneControlMode_Left',
										LeftControlLaneState(),
										transitions={'done': 'LaneControlMode', 'proceed': 'LaneControlMode_Left'},
										autonomy={'done': Autonomy.Off, 'proceed': Autonomy.Off})

			# x:898 y:386
			OperatableStateMachine.add('ObstacleDetectorMode',
										ObstacleDetectorState(),
										transitions={'obstacle': 'ObstacleDetectorMode', 'done': 'LaneControlMode'},
										autonomy={'obstacle': Autonomy.Off, 'done': Autonomy.Off})

			# x:909 y:508
			OperatableStateMachine.add('ParkingMode',
										ParkingState(),
										transitions={'parking_proceed': 'ParkingMode', 'done': 'LaneControlMode'},
										autonomy={'parking_proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:952 y:671
			OperatableStateMachine.add('StopBarMode',
										StopBarState(),
										transitions={'proceed': 'StopBarMode', 'done': 'LaneControlMode'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:179 y:249
			OperatableStateMachine.add('TrafficSignMode',
										TrafficSign(),
										transitions={'intersection_sign': 'IntersectionMode', 'obstacle_detection': 'ObstacleDetectorMode', 'parking_detection': 'ParkingMode', 'stop_bar_status': 'StopBarMode', 'tunnel_info': 'TunnelMode'},
										autonomy={'intersection_sign': Autonomy.Off, 'obstacle_detection': Autonomy.Off, 'parking_detection': Autonomy.Off, 'stop_bar_status': Autonomy.Off, 'tunnel_info': Autonomy.Off})

			# x:808 y:788
			OperatableStateMachine.add('TunnelMode',
										TunnelState(),
										transitions={'proceed': 'TunnelMode', 'done': 'LaneControlMode'},
										autonomy={'proceed': Autonomy.Off, 'done': Autonomy.Off})

			# x:894 y:260
			OperatableStateMachine.add('IntersectionMode',
										IntersectionState(),
										transitions={'turn_left': 'LaneControlMode_Left', 'turn_right': 'LaneControlMode'},
										autonomy={'turn_left': Autonomy.Off, 'turn_right': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
