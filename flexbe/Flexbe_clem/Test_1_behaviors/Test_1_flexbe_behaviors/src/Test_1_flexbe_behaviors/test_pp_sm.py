#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Test_1_flexbe_states.IncrementModuloState import IncrementModuloState
from Test_1_flexbe_states.MoveBaseState_2 import MoveBaseState_2
from Test_1_flexbe_states.SendFourState import SendNumberState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 10 2025
@author: Clem
'''
class Test_PPSM(Behavior):
	'''
	Petite pute
	'''


	def __init__(self):
		super(Test_PPSM, self).__init__()
		self.name = 'Test_PP'

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
			# x:46 y:60
			OperatableStateMachine.add('send',
										SendNumberState(number=4),
										transitions={'done': 'inc'},
										autonomy={'done': Autonomy.Off},
										remapping={'number': 'number'})

			# x:319 y:124
			OperatableStateMachine.add('move1',
										MoveBaseState_2(),
										transitions={'done': 'inc', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'number': 'number'})

			# x:86 y:141
			OperatableStateMachine.add('inc',
										IncrementModuloState(),
										transitions={'done': 'move1'},
										autonomy={'done': Autonomy.Off},
										remapping={'number': 'number'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
