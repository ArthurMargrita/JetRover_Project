#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Jetauto_final_flexbe_states.MoveBarState import MoveBarState
from Jetauto_final_flexbe_states.MoveTableState import MoveTableState
from Jetauto_final_flexbe_states.OrderReceptionState_3 import OrderReceptionState_3
from Jetauto_final_flexbe_states.Speaking import Speaking
from Jetauto_final_flexbe_states.Speaking_2 import Speaking_2
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 20 2025
@author: arthur
'''
class Final_commandSM(Behavior):
	'''
	final command
	'''


	def __init__(self):
		super(Final_commandSM, self).__init__()
		self.name = 'Final_command'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:434 y:297
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:169 y:99
			OperatableStateMachine.add('check order',
										OrderReceptionState_3(),
										transitions={'move': 'speak 2 order', 'verif_2': 'speak no cmd', 'erreur': 'speak wrong cmd'},
										autonomy={'move': Autonomy.Off, 'verif_2': Autonomy.Off, 'erreur': Autonomy.Off},
										remapping={'goal_pose_key': 'goal_pose_key', 'command': 'command', 'table': 'table'})

			# x:571 y:172
			OperatableStateMachine.add('move to bar',
										MoveBarState(),
										transitions={'Bar': 'speak get bar', 'failed': 'failed'},
										autonomy={'Bar': Autonomy.Off, 'failed': Autonomy.Off})

			# x:488 y:447
			OperatableStateMachine.add('move to table',
										MoveTableState(),
										transitions={'Table': 'speak get table', 'failed': 'failed'},
										autonomy={'Table': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_pose_key': 'goal_pose_key'})

			# x:370 y:107
			OperatableStateMachine.add('speak 2 order',
										Speaking_2(texte=""going to table {table} for {command}"", pause=2.0),
										transitions={'done': 'move to bar'},
										autonomy={'done': Autonomy.Off},
										remapping={'command': 'command', 'table': 'table'})

			# x:615 y:301
			OperatableStateMachine.add('speak get bar',
										Speaking(data="i am taking the glass", duration=2.0),
										transitions={'done': 'move to table'},
										autonomy={'done': Autonomy.Off})

			# x:251 y:485
			OperatableStateMachine.add('speak get table',
										Speaking(data="here is your order", duration=2.0),
										transitions={'done': 'check order'},
										autonomy={'done': Autonomy.Off})

			# x:115 y:211
			OperatableStateMachine.add('speak no cmd',
										Speaking(data="no command in the list", duration=2.0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:393 y:27
			OperatableStateMachine.add('speak wrong cmd',
										Speaking(data="wrong command", duration=2.0),
										transitions={'done': 'check order'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
