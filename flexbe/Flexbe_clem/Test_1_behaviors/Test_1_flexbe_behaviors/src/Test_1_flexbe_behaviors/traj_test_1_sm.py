#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Test_1_flexbe_states.DemandeState import DemandeState
from Test_1_flexbe_states.MoveBaseState import MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 26 2025
@author: Clem
'''
class Traj_test_1SM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(Traj_test_1SM, self).__init__()
		self.name = 'Traj_test_1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:128 y:300, x:491 y:303
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:252 y:102
			OperatableStateMachine.add('Dmde',
										DemandeState(),
										transitions={'done': 'Move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose'})

			# x:226 y:231
			OperatableStateMachine.add('Move',
										MoveBaseState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose', 'final_pose': 'final_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
