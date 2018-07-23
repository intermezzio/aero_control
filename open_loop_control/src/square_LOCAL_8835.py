from transition_control import *

if __name__ == '__main__':
    controller = TranslationController(MANEUVER_VELOCITY_SETPOINT, MANEUVER_REFERENCE_FRAME, MANEUVER_DURATION)

	    #Square ######################
    fin = 'bu'               	#
    velsp__fin = []
    duration = []

	velsp__fin.append([0,1, 0])  #
	duration.append(2)       	#
	                         	#
	velsp__fin.append([0,0,1])   #
	duration.append(2)       	#
	                         	#
	velsp__fin.append([0,0,1])   #
	duration.append(2)       	#

	velsp__fin.append([0,-1,0])
	duration.append(2)

	velsp__fin.append([0,0,-1])   #
	duration.append(2)        	#
	###############################

	    # Iterate through maneuvers
	for i in range(len(steps)):
	    execute_maneuver(velsp__fin[i], fin, duration[i])
	rospy.loginfo(velsp__fin[i])
	rospy.spin()

	controller.stop_streaming_offboard_points()
	print('DONE!')
