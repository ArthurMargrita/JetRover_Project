#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Test_1_flexbe_states.move_forward_state import MoveForwardFor5SecState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Feb 06 2025
@author: Clem
'''
class Test_1SM(Behavior):
	'''
	Suivi d'un test
	'''


	def __init__(self):
		super(Test_1SM, self).__init__()
		self.name = 'Test_1'

		# parameters of this behavior
		self.add_parameter('waiting_time', 2)
		self.add_parameter('vitesse', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hello = "Hello world"
		# x:71 y:303, x:436 y:321
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:76
			OperatableStateMachine.add('Initial_wait',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'Print_greeting'},
										autonomy={'done': Autonomy.Off})

			# x:214 y:183
			OperatableStateMachine.add('Print_greeting',
										LogState(text=hello, severity=2),
										transitions={'done': 'a'},
										autonomy={'done': Autonomy.High})

			# x:198 y:346
			OperatableStateMachine.add('a',
										MoveForwardFor5SecState(speed=self.vitesse),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
