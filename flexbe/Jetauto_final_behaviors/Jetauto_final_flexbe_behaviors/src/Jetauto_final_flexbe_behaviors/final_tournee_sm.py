#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Jetauto_final_flexbe_states.CheckOrder_State_output import CheckOrder_State_output
from Jetauto_final_flexbe_states.CheckPark import CheckPark
from Jetauto_final_flexbe_states.GetNextTablePositionState import GetNextTablePositionState
from Jetauto_final_flexbe_states.Move_point_to_point_2 import Move_point_to_point_2
from Jetauto_final_flexbe_states.SendNumberState import SendNumberState
from Jetauto_final_flexbe_states.Speaking import Speaking
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 20 2025
@author: arthur
'''
class Final_tourneeSM(Behavior):
	'''
	tournee final
	'''


	def __init__(self):
		super(Final_tourneeSM, self).__init__()
		self.name = 'Final_tournee'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 964 88 
		# Tables:|n|n[[0, 0, 0, 0], [1, 0.20, 1.35, 90], [2, -1.96, 1.46, 90], [3, -3, -1.30, -90], [4, 0.33, -1.39, -90]]|n|nla premiere liste (0) étant le park|n---------------------------|n|nIP robot:|n"192.168.149.71"



	def create(self):
		# x:868 y:385, x:345 y:260
		_state_machine = OperatableStateMachine(outcomes=['command', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('send first table',
										SendNumberState(last_table=1),
										transitions={'done': 'get next table pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'last_table': 'last_table'})

			# x:362 y:491
			OperatableStateMachine.add('check park',
										CheckPark(),
										transitions={'park': 'speak park', 'nonpark': 'speak glass'},
										autonomy={'park': Autonomy.Off, 'nonpark': Autonomy.Off},
										remapping={'last_table': 'last_table'})

			# x:221 y:53
			OperatableStateMachine.add('get next table pose',
										GetNextTablePositionState(poses=[[0, 0, 0, 0], [1, 0.20, 1.35, 90], [2, -1.96, 1.46, 90], [3, -3, -1.30, -90], [4, 0.33, -1.39, -90]]),
										transitions={'position': 'move', 'failed': 'failed'},
										autonomy={'position': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'last_table': 'last_table', 'x': 'x', 'y': 'y', 'table': 'last_table', 'angle': 'angle'})

			# x:470 y:76
			OperatableStateMachine.add('move',
										Move_point_to_point_2(),
										transitions={'done': 'speak order', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x': 'x', 'y': 'y', 'angle': 'angle'})

			# x:189 y:410
			OperatableStateMachine.add('speak glass',
										Speaking(data="no glasses", duration=2.0),
										transitions={'done': 'get next table pose'},
										autonomy={'done': Autonomy.Off})

			# x:477 y:375
			OperatableStateMachine.add('speak no order',
										Speaking(data="checkin order", duration=2.0),
										transitions={'done': 'check park'},
										autonomy={'done': Autonomy.Off})

			# x:668 y:161
			OperatableStateMachine.add('speak order',
										Speaking(data="checking order", duration=2.0),
										transitions={'done': 'check order'},
										autonomy={'done': Autonomy.Off})

			# x:564 y:535
			OperatableStateMachine.add('speak park',
										Speaking(data="i am charging", duration=2.0),
										transitions={'done': 'get next table pose'},
										autonomy={'done': Autonomy.Off})

			# x:656 y:285
			OperatableStateMachine.add('check order',
										CheckOrder_State_output(ip_server_url="192.168.4.122"),
										transitions={'command': 'command', 'empty': 'speak no order', 'failed': 'failed'},
										autonomy={'command': Autonomy.Off, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'table': 'table', 'commande': 'commande'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
