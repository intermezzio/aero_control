from transition_control import *

if __name__ == '__main__':
    controller = TranslationController(MANEUVER_VELOCITY_SETPOINT, MANEUVER_REFERENCE_FRAME, MANEUVER_DURATION)

    ## Shape: https://i.etsystatic.com/15395627/r/il/4ac176/1422965528/il_340x270.1422965528_mbtj.jpg
    fin = 'fc'
    velsp__fin = list()
    duration = list()
    ## Steps:

    # Step 1: Left Side Bottom
    velsp__fin.append([-1,-1,0])
    duration.append(3)

    # Step 2: Left Side Top
    velsp__fin.append([1,-1,0])
    duration.append(1)

    # Step 3: Left Side Inner Top
    velsp__fin.append([1,1,0])
    duration.append(1)

    # Step 4: Left Side Inner Top To Middle
    velsp__fin.append([1,-1,0])
    duration.append(1)

    # Step 5: Right Side Inner Top To Middle
    velsp__fin.append([1,1,0])
    duration.append(1)

    # Step 6: Right Side Inner Top
    velsp__fin.append([1,-1,0])
    duration.append(1)

    # Step 7: Right Side Top
    velsp__fin.append([1,1,0])
    duration.append(1)

    # Step 8: Across Middle
    velsp__fin.append([-1,0,0])
    duration.append(6)

    # Step 9: Back Up Left
    velsp__fin.append([1,-1,0])
    duration.append(1)

    # Step 10: Across Top
    velsp__fin.append([1,0,0])
    duration.append(4)

    # Step 11: Back Down Right
    velsp__fin.append([1,1,0])
    duration.append(1)

    # Step 12: Right Side Bottom
    velsp__fin.append([-1,-1,0])
    duration.append(3)

    # Step 13: Right Side Inner
    velsp__fin.append([1/3,-1,0])
    duration.append(3)

    # Step 14: Small Across Middle
    velsp__fin.append([-1,0,0])
    duration.append(2)

    # Step 15: Left Side Inner
    velsp__fin.append([1/3,1,0])
    duration.append(3)

    # Iterate through maneuvers
    for i in range(len(steps)):
        execute_maneuver(velsp__fin[i], fin, duration[i])

    rospy.spin()

    controller.stop_streaming_offboard_points()
    print('DONE!')
