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
from Test_1_flexbe_states.SendFourState import SendFourState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 03 2025
@author: Clem
'''
class Test_traj_2SM(Behavior):
	'''
	Test traj via cmde appli
	'''


	def __init__(self):
		super(Test_traj_2SM, self).__init__()
		self.name = 'Test_traj_2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:534 y:171, x:130 y:297
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:288 y:115
			OperatableStateMachine.add('azer',
										SendFourState(),
										transitions={'done': 'azertayertater'},
										autonomy={'done': Autonomy.Off},
										remapping={'number': 'number'})

			# x:282 y:263
			OperatableStateMachine.add('azertayertater',
										IncrementModuloState(),
										transitions={'done': 'pkoihuygf'},
										autonomy={'done': Autonomy.Off},
										remapping={'number': 'number'})

			# x:65 y:156
			OperatableStateMachine.add('pkoihuygf',
										MoveBaseState_2(),
										transitions={'done': 'azertayertater', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'number': 'number'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
