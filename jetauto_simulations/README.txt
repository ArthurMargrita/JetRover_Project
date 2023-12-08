1) modification du fichier .typerc appelé dans le .bashrc:


LIDAR_TYPE=A1
DEPTH_CAMERA_TYPE=Dabai
MACHINE_TYPE=jetauto_arm
HOST=ubuntu
MASTER=robot_JetRoverPro
LANGUAGE=French


2) j'ai utilisé le packet jetauto_description fourni par hiwonder sans toucher aux URDF

3) j'ai changé le fichier gazebo.launch pour appeler le fichier de configuration ROS_CONTROLLER.yaml

4) j'ai ajouté les pid dans le fichier de configuration ROS_CONTROLLER.yaml

5) j'ai lancé la simulation en ouvrant le fichier room_worlds.launch en enlevant le chargement du robot dans ce fichier

6) j'ai essayé de modifier les joints du fichier de configs mais ça ne marche pas puisque je ne vois pas les topics assocés au bras et au gripper
