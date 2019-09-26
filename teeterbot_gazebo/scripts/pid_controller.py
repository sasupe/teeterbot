#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import tf
import numpy as np

pub_left = rospy.Publisher('/teeterbot/left_speed_cmd', Float64, queue_size=10)
pub_right = rospy.Publisher('/teeterbot/right_speed_cmd', Float64, queue_size=10)

error_sum = 0;
prev_error = 0;
set_angle = 0;

def controller(data):
	global pub_left
	global pub_right
	global error_sum
	global prev_error
	global set_angle

	quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	
	error =  set_angle*np.pi/180 - pitch
	
	print (error*180/np.pi, error_sum)	

	if abs(error_sum) < 3:
		error_sum = error_sum + error;	

	if abs(error_sum) < 0.0:
		error_sum = 0;


	error_diff = error - prev_error;
	prev_error = error;

	kp = -20;
	ki = -2;
	kd = -0;
	target_speed = kp*error + ki * error_sum + kd * error_diff;


	if abs(error) > 0*np.pi/180:
		pub_left.publish(target_speed)
		pub_right.publish(target_speed)

	if abs(error) > 5*np.pi/180:
		pub_left.publish(0)
		pub_right.publish(0)
		error_sum = 0;
		error_diff = 0;
		prev_error =0

def target(data):
	global set_angle
	set_angle = data.data;
	

def listener():

    rospy.init_node('teeterbot_pid_controller', anonymous=True)

    rospy.Subscriber("/gazebo/teeterbot_base_link", Pose , controller)
    rospy.Subscriber("/set_angle", Float64 , target)

    rospy.spin()

if __name__ == '__main__':
    listener()
