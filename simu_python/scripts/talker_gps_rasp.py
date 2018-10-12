# !/usr/bin/env python
#
# talker qui publie les coordonnées gps de 
# la raspberry
#
 import rospy

 from std_msgs.msg import String

 def talker_gps_rasp():
 	pub = rospy.Publisher('chat_gps_rasp', String, queue_size =100)
 	rospy.init_node('talk_gps_rasp', anonymous = True)
 	rate = rospy.Rate(10)
 	while not rospy.is_shutdown():
 		gps_string = rospy.get_time()
 		rospy.loginfo(gps_string)
 		pub.publish(gps_string)
 		rate.sleep()

if __name__ == '__main__':
	try :
		talker_gps_rasp()
	except rospy.ROSInterruptException: 
		pass