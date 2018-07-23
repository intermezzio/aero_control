import translation_control as tc

if __name__ == '__main__':
    controller = tc.TranslationController(tc.MANEUVER_VELOCITY_SETPOINT, tc.MANEUVER_REFERENCE_FRAME, tc.MANEUVER_DURATION)
    
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
    controller.execute_maneuver(velsp__fin[i], fin, duration[i])

<<<<<<< HEAD
	    # Iterate through maneuvers
	for i in range(len(steps)):
	    execute_maneuver(velsp__fin[i], fin, duration[i])
	rospy.loginfo(velsp__fin[i])
	rospy.spin()

	controller.stop_streaming_offboard_points()
	print('DONE!')
=======
rospy.spin()

controller.stop_streaming_offboard_points()
print('DONE!')

>>>>>>> 4a12165284cb4e7409de36e3fef255c195e9dd5e
