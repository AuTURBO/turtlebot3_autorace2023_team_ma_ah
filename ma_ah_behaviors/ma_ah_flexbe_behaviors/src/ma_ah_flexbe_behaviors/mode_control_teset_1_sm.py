#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from test_sion_flexbe_states.cross_road_state import CrossroadState
from test_sion_flexbe_states.lane_control import ControlLaneState
from test_sion_flexbe_states.obstacle_detector_state import ObstacleDetectorState
from test_sion_flexbe_states.parking_state import ParkingState
from test_sion_flexbe_states.stop_bar_state import StopBarState
from test_sion_flexbe_states.traffic_light import TrafficLightState
from test_sion_flexbe_states.traffic_sign import TrafficSign
from test_sion_flexbe_states.tunnel_state import TunnelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Sep 25 2023
@author: sion jeon
'''
class mode_control_teset_1SM(Behavior):
	'''
	hello
	'''


	def __init__(self):
		super(mode_control_teset_1SM, self).__init__()
		self.name = 'mode_control_teset_1'

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
			# x:77 y:39
			OperatableStateMachine.add('TrafficLightMode',
										TrafficLightState(),
										transitions={'green_light': 'LaneControlMode', 'red_or_yellow_light': 'TrafficLightMode', 'no_signal': 'TrafficLightMode'},
										autonomy={'green_light': Autonomy.Off, 'red_or_yellow_light': Autonomy.Off, 'no_signal': Autonomy.Off})

			# x:398 y:48
			OperatableStateMachine.add('LaneControlMode',
										ControlLaneState(MAX_VEL=0.5),
										transitions={'done': 'LaneControlMode', 'failed': 'LaneControlMode', 'traffic_sign': 'TrafficSignMode'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'traffic_sign': Autonomy.Off})

			# x:1026 y:122
			OperatableStateMachine.add('ObsDetectorMode',
										ObstacleDetectorState(),
										transitions={'obstacle': 'ObsDetectorMode', 'no_obstacle': 'LaneControlMode'},
										autonomy={'obstacle': Autonomy.Off, 'no_obstacle': Autonomy.Off})

			# x:1185 y:279
			OperatableStateMachine.add('ParkingMode',
										ParkingState(),
										transitions={'parking_spot_available': 'ParkingMode', 'no_parking_spot': 'LaneControlMode'},
										autonomy={'parking_spot_available': Autonomy.Off, 'no_parking_spot': Autonomy.Off})

			# x:1109 y:422
			OperatableStateMachine.add('StopBarMode',
										StopBarState(),
										transitions={'stop': 'LaneControlMode', 'proceed': 'StopBarMode'},
										autonomy={'stop': Autonomy.Off, 'proceed': Autonomy.Off})

			# x:351 y:218
			OperatableStateMachine.add('TrafficSignMode',
										TrafficSign(),
										transitions={'lane_control': 'LaneControlMode', 'crossroad_sign': 'CrossRoadMode', 'obstacle_detection': 'ObsDetectorMode', 'parking_detection': 'ParkingMode', 'stop_bar_status': 'StopBarMode', 'tunnel_info': 'TunnelMode'},
										autonomy={'lane_control': Autonomy.Off, 'crossroad_sign': Autonomy.Off, 'obstacle_detection': Autonomy.Off, 'parking_detection': Autonomy.Off, 'stop_bar_status': Autonomy.Off, 'tunnel_info': Autonomy.Off})

			# x:726 y:518
			OperatableStateMachine.add('TunnelMode',
										TunnelState(),
										transitions={'proceed': 'TunnelMode', 'finished': 'LaneControlMode'},
										autonomy={'proceed': Autonomy.Off, 'finished': Autonomy.Off})

			# x:853 y:11
			OperatableStateMachine.add('CrossRoadMode',
										CrossroadState(),
										transitions={'turn_left': 'CrossRoadMode', 'turn_right': 'CrossRoadMode', 'no_sign': 'LaneControlMode'},
										autonomy={'turn_left': Autonomy.Off, 'turn_right': Autonomy.Off, 'no_sign': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
