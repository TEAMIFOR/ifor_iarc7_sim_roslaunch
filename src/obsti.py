#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import threading
def velgiv(i):
	pub=rospy.Publisher('/robot{}/cmd_vel'.format(i),Twist,queue_size=10)
	subdiv=Twist()

	rate=rospy.Rate(20)

	while not rospy.is_shutdown():
		subdiv.angular.x=subdiv.angular.y=subdiv.linear.z=0
		subdiv.linear.x=1
		subdiv.angular.z=0.2
		pub.publish(subdiv)

def main ():
	rospy.init_node("cmd_vel",anonymous=True)
	for i in range(10,14):
		t=threading.Thread(target=velgiv,args=(i,))
		t.start()
	
if __name__ =="__main__":
	main()
