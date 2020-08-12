#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

'''
turtlesim has these topics

/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

##turtle1/color_sensor message type is turtlesim/Color
uint8 r
uint8 g
uint8 b

##turtle1/pose message type is turtlesim/Pose
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity

'''

class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd = Twist()
        self.pose_msg = Pose()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def pose_callback(self, msg):
        self.pose_msg = msg


    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def interpolate(self, x, in_min, in_max, out_min, out_max):
    	return ((x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min)

    def move_straight(self, speed):

        # Initilize velocities
        self.cmd.linear.x = speed
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
        	self.cmd.linear.x = speed
        elif motion == "backward":
        	self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        #print(self.pose_msg)
        return s

    def move_straight_distance(self, motion, speed, move_distance):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
        	self.cmd.linear.x = speed
        elif motion == "backward":
        	self.cmd.linear.x = - speed

        distanceCovered = 0
        t0= time.time()
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (distanceCovered <= move_distance):

            t1=time.time()
            distanceCovered= abs(speed*(t1-t0))
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " " + str(move_distance) + " meters"
        print(self.pose_msg)
        return s

    def turn_angle(self, direction, speed, angle):
    	currentAngle = math.degrees(self.pose_msg.theta)

    	if direction == "clockwise":
            angleToBe = int(currentAngle - angle)
        else:
            angleToBe = int(currentAngle + angle)

        angelRange = list(range(0,181)) + list(range(-179,0))
    	difference = abs(currentAngle - angelRange[angleToBe])

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if direction == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed


        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while True:
        	difference = abs(math.degrees(self.pose_msg.theta) - angelRange[angleToBe])
        	if direction == "clockwise":
        		self.cmd.angular.z = -speed#*difference*0.05
        	else:
        		self.cmd.angular.z = speed#*difference*0.05
        	self.vel_publisher.publish(self.cmd)

        	if difference <0.4:
        		print("break condition")
        		break

        self.stop_robot() # set velocity to zero to stop the robot
        s = "Turned robot " + str(angle) + "degree "+ direction
        #print(s)       
        #print(self.pose_msg)


    def goTo(self, xCo, yCo):
    	time.sleep(1)
    	xCurr = self.pose_msg.x
    	yCurr = self.pose_msg.y
    	thetaCurr = math.degrees(self.pose_msg.theta)

    	x = xCo-xCurr
    	y = yCo-yCurr

    	angleToRotate =math.degrees(math.atan2(y,x)) - thetaCurr  #returns between -359 to +360
    	if angleToRotate <0:						#negative angle means clockwise rotation
    		angleToRotate = - angleToRotate
    		if angleToRotate <= 180:
    			self.turn_angle('clockwise', 1, angleToRotate)
    		else:
    			angleToRotate = 360 -angleToRotate	#taking supplementery angle
    			self.turn_angle('anticlockwise', 1, angleToRotate)
    	else:										#positive angle means anticlockwise rotation
    		if angleToRotate <= 180:
    			self.turn_angle('anticlockwise', 1, angleToRotate)
    		else:
    			angleToRotate = 360 -angleToRotate	#taking supplementery angle
    			self.turn_angle('clockwise', 1, angleToRotate)
		
    	dist = math.sqrt((xCurr-xCo)**2+(yCurr-yCo)**2)

    	while (dist >=0.1):
    		xCurr = self.pose_msg.x
    		yCurr = self.pose_msg.y   		
    		dist = math.sqrt((xCurr-xCo)**2+(yCurr-yCo)**2)
    		self.move_straight(1.1*dist) # moving staright to start with proportional control
    		
    	self.stop_robot()

    	angleAdjastment = math.degrees(0 - self.pose_msg.theta)
    	self.turn_angle('anticlockwise', 1, angleAdjastment)
    	#print(self.pose_msg)

    def clean_straight(self):

    	self.goTo(1,1)
    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('anticlockwise', 2, 90)

    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 2)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('clockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 2)
    	self.turn_angle('clockwise', 2, 90)

    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 2)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('clockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 2)
    	self.turn_angle('clockwise', 2, 90)

    	self.move_straight_distance('forward', 3, 9)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 1.5)
    	self.turn_angle('anticlockwise', 2, 90)
    	self.move_straight_distance('forward', 3, 9)
    	self.goTo(1,1)
    	print("Complete...")

    def clean_spiral(self):
    	constantSpeed = 4
    	rk = 0.5
    	while (self.pose_msg.x<10.5 and self.pose_msg.y <10.5):
    		rk = rk+1
	    	self.cmd.linear.x = rk
	        self.cmd.linear.y = 0
	        self.cmd.linear.z = 0
	        self.cmd.angular.x = 0
	        self.cmd.angular.y = 0
	        self.cmd.angular.z = constantSpeed #vk/(0.5+rk)
	        self.vel_publisher.publish(self.cmd)
	        self.rate.sleep()
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.vel_publisher.publish(self.cmd)

robot = RobotControl()

# print(robot.move_straight_time('forward', 2, 2))
# robot.turn_angle("anticlockwise", 1, 90)
#print(robot.move_straight_time('forward', 1, 2))

robot.clean_straight()
#robot.clean_spiral()