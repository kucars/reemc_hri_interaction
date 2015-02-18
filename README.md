# Reem Audio Visual Interaction
Audio Visual HRI Interaction with Reemc Robot from Pal Robotics

## Requirements and dependencies
1. Python 2.x
2. Opencv 2.x/3.x
3. Openni
4. sox, a command line sound processing tool
5. sound localization package ("pal-asr packages")
6. wolframalpha python package if your planning to use interact node (pip install wolframalpha)

##How to run
It is recommended that you run the commands in the specified order.

using the NAO robot:

1. roslaunch nao_driver nao_driver.launch
2. rosservice call /body_stiffness enable			          #enable joints stiffness
3. roslaunch pal_face_detector_opencv detector.launch		#make sure that you specify the right camera frames topic
4. rosrun pal_jackproc jackDeployer --device=hw:1,0		  #make sure that you change the audio card ID
5. rosrun pal_sound_loc pal_sound_loc
6. roslaunch fusion fusion.launch					              #make sure that you specify the right camera frames topic
7. roslaunch particle_tracker tracker.launch			      #make sure that you specify the right camera frames topic
8. rosrun interact gspeech

using the REEMC simulation:
'''
roslaunch  reemc_gazebo small_office.launch			    #launch the gazebo simulation along with the reem sim model
roslaunch fusion face_detector.launch		            #make sure that you specify the right camera frames topic
rosrun pal_jackproc jackDeployer --device=hw:1,0		  #make sure that you change the audio card ID
rosrun pal_sound_loc pal_sound_loc
roslauch fusion fusion.launch		                    #make sure that you specify the right camera frames topic
roslauch particle_tracker tracker.launch			        #make sure that you specify the right camera frames topic
rosrun interact gspeech
'''
Notes:
======
* The interaction nodes implement client-server model in which the "interact speech" node issue requests to the "data fusion" and the "particle tracker" nodes. Therefore it is important to run the three nodes mention in the specified order.
* if you change an argument in any of the launch files used in steps 5-7, make sure that you modify the other launch files accordingly
