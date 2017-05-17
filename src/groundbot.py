#! /usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi,sin,asin
from time import time
import threading 
splt=[]
s=''

def callback(data):
	global s
	global splt
	s=data.data
	splt=s.split(" ")

	 	
def velgive(i):
	pub=rospy.Publisher('/robot{}/cmd_vel'.format(i),Twist,queue_size=10)
	subdiv=Twist()
	count=0
	rate=rospy.Rate(20)
	while not rospy.is_shutdown():
		now=rospy.get_rostime()
		change_angle=random.randint(0,20)
		change_angle_rad=change_angle*pi/180
		if (now.secs!=0 ):
			if (now.secs%5==0 and now.secs%20!=0 and splt[i]!='1'):
				subdiv.angular.x=subdiv.angular.y=subdiv.linear.y=subdiv.linear.z=0
				subdiv.linear.x=0.33
				subdiv.angular.z=0.33
				change_time=change_angle_rad/0.33
				current_time=time()
				required_time=current_time+(change_time)
				while (time()<required_time):
					pub.publish(subdiv)
				rospy.loginfo(s)
				rate.sleep()
			elif(now.secs%20==0 and splt[i]!='1'):
				subdiv.linear.x=subdiv.linear.y=subdiv.linear.z=subdiv.angular.x=subdiv.angular.y=0
				subdiv.angular.z=1
				current_time=time()
				required_time=current_time+(pi/1)
				while(time()<required_time):
					pub.publish(subdiv)
				rospy.loginfo(s)
				rate.sleep()
			elif(splt[i]=='1'):
				subdiv.linear.x=subdiv.linear.y=subdiv.linear.z=subdiv.angular.x=subdiv.angular.y=0
				subdiv.angular.z=1
				current_time=time()
				required_time=current_time+(pi/1)
				while(time()<required_time):
					pub.publish(subdiv)
				rospy.loginfo(s)
				rate.sleep()

			subdiv.angular.z=subdiv.angular.x=subdiv.angular.y=subdiv.linear.y=subdiv.linear.z=0
			subdiv.linear.x=0.33
			rospy.loginfo(s)
			pub.publish(subdiv)
			rate.sleep()

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
	rospy.Subscriber("/contact",String,callback)
	while len(splt)==0:
		continue
	for i in range(0,14):
		if (i<10):
			t=threading.Thread(target=velgive,args=(i,))
		else:
			t=threading.Thread(target=velgiv,args=(i,))
		#t.daemon=True
		t.start()
		
if __name__ =="__main__":
	main()


