import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
from std_srvs.srv import Empty
import sys

def turtle_circle(radius, name = 'turtle1'):
	pub = rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)
	vel = Twist()
	
	vel.linear.x = radius
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = 1
	
	rospy.loginfo("Radius = %f", radius)
	pub.publish(vel)
	rate.sleep()


def turtle_set_pen(red, green, blue, pen_width, pen_up, name = 'turtle1'):
	rospy.wait_for_service(f'{name}/set_pen')
	try:
		srv_p = rospy.ServiceProxy(f'{name}/set_pen', SetPen)
		res_m = srv_p(int(red), int(green), int(blue), int(pen_width), int(pen_up))
	except rospy.ServiceException as e:
		print('Turtle Set Pen Service failed.')
		print(e)
		

def turtle_kill(name):
	srv_name = 'kill'
	rospy.wait_for_service(srv_name)
	try:
		srv_p = rospy.ServiceProxy(srv_name, Kill)
		res_m = srv_p(name)


	except rospy.ServiceException as e:
		print('Turtle Reset Service failed.')
		print(e)


def turtle_reset():
	srv_name = 'reset'
	rospy.wait_for_service(srv_name)
	try:
		srv_p = rospy.ServiceProxy(srv_name, Empty)
		res_m = srv_p()
	except rospy.ServiceException as e:
		print('Turtle Reset Service failed.')
		print(e)


def turtle_spawn(x, y, theta, name):
	srv_name = 'spawn'
	rospy.wait_for_service(srv_name)
	try:
		srv_p = rospy.ServiceProxy(srv_name, Spawn)
		res_m = srv_p(float(x), float(y), float(theta), name)
	except rospy.ServiceException as e:
		print('Turtle Spawn Service failed: ')
		print(e)


if __name__ == '__main__':
	try:
		rospy.init_node('turtle_mover')

		turtle_reset()
		turtle_spawn(5, 5, 90, 'hello')
		turtle_spawn(5, 6, 270, 'hell')

		turtle_set_pen(255, 0, 255, 5, 0)
		turtle_set_pen(255, 0, 0, 10, 0, 'hello')
		turtle_set_pen(0, 255, 255, 20, 0, 'hell')

		while not rospy.is_shutdown():
			turtle_circle(4)
			turtle_circle(3, 'hello')
			turtle_circle(2, 'hell')
	except rospy.ROSInterruptException as e:
		print('ROS Interrupt Exception: ')
		print(e)
