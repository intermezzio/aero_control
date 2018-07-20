#code notes for intermediate challenge module 1



#flight formation
#given info
	#Heading angle (HA)
	#Tag 1 coordinates (NED) (y1,z1)
	#Tag 2 coordinates (NED) (y2,z2)
#required info
	#Midpt ((z1+z2)/2,(y1+y2)/2)
	#Distance/depth @ midpoint

#Steps
#1) Compute midpoint
#2) Translate to midpt, maintain x
#3) Compute depth d: d = (sqrt((z2-(z1+z2)/2)^2 + (y2-(y1+y2)/2)^2)/tan(HA/2)
#4) Translate to x, maintain y,z

import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Twist

#get distance Tag1

#get distance Tag2


HA = #HEADING ANGLE(GIVEN)


class FlightForm:
	def __init__(self):

		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, self.data_collect)

	def data_collect(self,msg):

		if len(msg.markers) == 2:
			marker_close = min(msg.markers)
			marker_far = max(msg.markers)

		marker_close_x = marker_close.pose.pose.position.x
		marker_close_y = marker_close.pose.pose.position.y
		marker_far_x = marker_far.pose.pose.position.x
		marker_far_y = marker_far.pose.pose.position.y

		mcx = marker_close_x
		mcy = marker_close_y
		mfx = marker_far_x
		mfy = marker_far_y

		z_distance = marker_close.pose.pose.position.z

		self.calc()

	def calc(self):

		mdpt_x = ((mcx+mfx)/2)
		mdpt_y = ((mcy+mfy)/2)

		depth_z = z_distance - (np.sqrt((mcx-mdpt_x)^2 + (mcy-mdpt_y)^2)/np.tan(HA/2))

		self.translate()

	def translate(self):

		#i forget the syntax for this code
		Twist.linear.x = mdpt_x
		Twist.linear.y = mdpt_y
		Twist.linear.z = depth_z

if __name__ == '__main__':
	rospy.init.node('intermed.py')
	a = FlightForm

	rospy.spin()



