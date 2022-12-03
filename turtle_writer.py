import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
from std_srvs.srv import Empty
import math
import Alphabets
import threading
import turtle_mover

class Turtle:
	def __init__(self, name, red = 255, green = 255, blue = 255,  width = 15):
		self.name = name
		self.red = red
		self.green = green
		self.blue = blue
		self.width = width
		self.pen_up = 0

		self.spawn()
		self.pub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=10)
		self.set_pen()
		self.rate = rospy.Rate(1)


	def move(self, l, a):
		vel = Twist()
		vel.linear.x = l
		vel.angular.z = a
		
		while not rospy.is_shutdown():
			if(self.pub.get_num_connections() > 0):
				self.pub.publish(vel)
				break


			self.rate.sleep()


		self.rate.sleep()


	def forward(self, l):
		self.move(l, 0)


	def left(self, a):
		self.move(0, math.radians(a))


	def right(self, a):
		self.move(0, math.radians(-a))


	def set_pen(self):
		rospy.wait_for_service(f'{self.name}/set_pen')
		try:
			srv_p = rospy.ServiceProxy(f'{self.name}/set_pen', SetPen)
			res_m = srv_p(int(self.red), int(self.green), int(self.blue), int(self.width), int(self.pen_up))
		

		except rospy.ServiceException as e:
			print('Turtle Set Pen Service failed.')
			print(e)


	def set_color(self, red, green, blue):
		self.red = red
		self.green = green
		self.blue = blue
		self.set_pen()


	def set_width(self, width):
		self.width = width
		self.set_pen()


	def penup(self):
		self.pen_up = 1;
		self.set_pen()


	def pendown(self):
		self.pen_up = 0;
		self.set_pen()


	def spawn(self):
		srv_name = 'spawn'
		rospy.wait_for_service(srv_name)
		try:
			srv_p = rospy.ServiceProxy(srv_name, Spawn)
			res_m = srv_p(float(1), float(4), float(0), self.name)


		except rospy.ServiceException as e:
			print('Turtle Spawn Service failed: ')
			print(e)


def Write_CSE():
	turtle = Turtle("t1")
	Alphabets.__C__(turtle, go, space)
	Alphabets.__S__(turtle, go, space)
	Alphabets.__E__(turtle, go, space)


def Draw_Circle():
	turtle_mover.turtle_spawn(1, 0, 0, 'turtle1')
	turtle_mover.turtle_spawn(10, 10, 0, 'turtle2')
	turtle_mover.turtle_set_pen(255, 0, 0, 5, 0, 'turtle2')
	turtle_mover.turtle_set_pen(0, 255, 0, 4, 0, 'turtle1')

	while not rospy.is_shutdown():
		turtle_mover.turtle_circle(5, 'turtle1')
		turtle_mover.turtle_circle(4, 'turtle2')


if __name__ == '__main__':
	go = 4
	space = 1

	try:
		rospy.init_node('turtle_writer')
		turtle_mover.turtle_reset()
		turtle_mover.turtle_kill('turtle1')

		t1 = threading.Thread(target = Write_CSE)
		t1.start()

		# turtle_mover.turtle_circle(4, 't1')
		t2 = threading.Thread(target = Draw_Circle)
		t2.start()

		t1.join()
		t2.join()


	except rospy.ROSInterruptException as e:
		print('ROS Interrupt Exception: ')
		print(e)

