#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from Test_1_flexbe_states.MoveState import MoveState
from Test_1_flexbe_states.OrderReceptionState import OrderReceptionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Mar 05 2025
@author: Clem
'''
class Test_1_TSM(Behavior):
	'''
	Test numéro 1 de la tournée de routine du robot
	'''


	def __init__(self):
		super(Test_1_TSM, self).__init__()
		self.name = 'Test_1_T'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		Park = "-1.5,-2.5,0.0,-0.7,0.7,0.0,0.0"
		Bar = "3.0,-1.0,0.0,0.0,0.0,0.0,1.0"
		Table_1 = "-4.0,-1.0,0.0,0.0,0.1,0.0,0.0"
		Table_2 = "-3.0,-1.0,0.0,-0.7,0.7,0.0,0.0"
		Table_3 = "-1.0,-1.0,0.0,-0.7,0.7,0.0,0.0"
		Table_4 = "1.0,-1.0,0.0,-0.7,0.7,0.0,0.0"
		# x:30 y:408, x:469 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:108
			OperatableStateMachine.add('lakfliazf',
										OrderReceptionState(),
										transitions={'done': 'Table_1', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose'})

			# x:246 y:58
			OperatableStateMachine.add('Table_1',
										MoveState(text=Table_1),
										transitions={'done': 'Table_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'final_pose': 'final_pose'})

			# x:470 y:60
			OperatableStateMachine.add('Table_2',
										MoveState(text=Table_2),
										transitions={'done': 'Table_3', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'final_pose': 'final_pose'})

			# x:690 y:138
			OperatableStateMachine.add('Table_3',
										MoveState(text=Table_3),
										transitions={'done': 'Table_4', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'final_pose': 'final_pose'})

			# x:553 y:297
			OperatableStateMachine.add('Table_4',
										MoveState(text=Table_4),
										transitions={'done': 'Park', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'final_pose': 'final_pose'})

			# x:211 y:278
			OperatableStateMachine.add('Park',
										MoveState(text=Park),
										transitions={'done': 'Table_1', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'final_pose': 'final_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
