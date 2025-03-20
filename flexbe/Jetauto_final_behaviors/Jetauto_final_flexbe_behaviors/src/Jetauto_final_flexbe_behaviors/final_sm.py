#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Jetauto_final_flexbe_behaviors.final_command_sm import Final_commandSM
from Jetauto_final_flexbe_behaviors.final_tournee_sm import Final_tourneeSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 20 2025
@author: arthur
'''
class FinalSM(Behavior):
	'''
	final
	'''


	def __init__(self):
		super(FinalSM, self).__init__()
		self.name = 'Final'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(Final_commandSM, 'Final_command')
		self.add_behavior(Final_tourneeSM, 'Final_tournee')

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
			# x:183 y:139
			OperatableStateMachine.add('Final_tournee',
										self.use_behavior(Final_tourneeSM, 'Final_tournee'),
										transitions={'command': 'Final_command', 'failed': 'failed'},
										autonomy={'command': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:469 y:162
			OperatableStateMachine.add('Final_command',
										self.use_behavior(Final_commandSM, 'Final_command'),
										transitions={'finished': 'Final_tournee', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
