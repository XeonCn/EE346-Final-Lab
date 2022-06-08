#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import Image
import cv2.aruco as aruco
from playsound import playsound



aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param = aruco.DetectorParameters_create()

class Follower:
	def __init__(self):

		self.bridge = cv_bridge.CvBridge()

		self.image_sub = rospy.Subscriber('camera/image',
										Image, self.image_callback)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',
										Twist, queue_size=1)
		self.starttimer = rospy.get_time()
		self.twist = Twist()
		self.image = None
		self.gray = None  # grey scale image for Aruco marker detection
		self.start = 0  # flag: enter the racetrack 
		self.right = 0  # flag: leave the racetrack 
		self.finish = 0  # flag: after leaving, move forward for a while
		self.turn1 = 0  # flag: left turn1 (close to p2)
		self.turn2 = 0
		self.turn3 = 0
		self.beginFollow = 1  # flag: lane follower
		self.markerID = None
		self.aruco_found = [False]*5
		self.audio_filename = ["0.mp3","1.mp3","2.mp3","3.mp3","4.mp3"]
		self.s0 = rospy.get_time() + 120  # time when turn0 is finished
		self.s1 = rospy.get_time() + 120  # time when turn1 is finished
		self.s2 = rospy.get_time() + 120  # time when turn2 is finished
		self.s3 = rospy.get_time() + 120  # time when turn3 is finished
        # add 120s since timers start before navigation, while navigation consumes much time, so timers are updated when required

	def image_callback(self, msg):
		# receive image
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

		# Aruco marker detecting
		corners, self.markerID, rejected = aruco.detectMarkers(self.gray, aruco_dict, parameters=param)
		if len(corners) > 0:
			aruco_ID = self.markerID.squeeze()
			if not self.aruco_found[aruco_ID]:
				self.aruco_found[aruco_ID] = True
				playsound("/home/ee368-14/catkin_ws/src/lane_turtlebot3/audios/"+self.audio_filename[aruco_ID],block=False)
				print(self.markerID)


	def run(self):
		if self.image is None:
			rospy.loginfo("Waiting image")
			rospy.sleep(1)
			return
		top_x = 68  # 102 73     62 10 140 120     50 20 140 120    68 5 140 120
		top_y = 5  # 10 26      40 30 140 120
		bottom_x = 140  # 230
		bottom_y = 120
		# selecting 4 points from the original image
		pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y],
							[160 - bottom_x, 120 + bottom_y]])

		# selecting 4 points from image that will be transformed
		pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

		# finding homography matrix
		h, status = cv2.findHomography(pts_src, pts_dst)

		lower_black = np.array([0, 0, 0])
		upper_black = np.array([180, 255, 80])
		hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		image_t = cv2.inRange(hsv, lower_black, upper_black)

		# homography process
		BEV = cv2.warpPerspective(image_t, h, (1000, 600))  # 10:6

		mask1 = np.copy(BEV)
		mask2 = np.copy(BEV)
		mask3 = np.copy(BEV)
		mask4 = np.copy(BEV)
		mask5 = np.copy(BEV)
		h = 600
		w = 1000

		mask1[:, 500:w] = 0  # left half
		mask2[:, 0:500] = 0  # right half
		mask3[100:h, :] = 0
		mask3[:, 500:w] = 0  # upper left corner
		mask4[400:h, :] = 0
		mask4[:, 0:500] = 0  # upper right corner
		mask5[300:h, :] = 0  # top half
		M0 = cv2.moments(BEV)
		M1 = cv2.moments(mask1)
		M2 = cv2.moments(mask2)
		M3 = cv2.moments(mask3)
		M4 = cv2.moments(mask4)
		M5 = cv2.moments(mask5)

		if self.beginFollow == 1:
			if self.start == 0 and M5['m00'] == 0:
				rospy.sleep(2)
				print("init")  # enter the racetrack
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(0.31)
				self.twist.linear.x = 0.0
				self.twist.angular.z = -1.0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.555)
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.45)
				self.twist.linear.x = 0.2
				self.twist.angular.z = 1.05
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(2.5)
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				self.start = 1

			t2 = rospy.get_time()
			# t2 - self.s_ > _ is to avoid continuous turning, since the image will not be updated immediately after turning command
			if M3['m00'] == 0 and self.finish == 0 and t2 - self.s0 > 15:
				if self.turn1 == 0:
					self.turn1 = 1
					self.turn2 = 1
					self.s1 = rospy.get_time()
					print('point2')
					self.beginFollow = 0  # quit lane following
				elif self.turn2 == 1 and t2 - self.s1 > 7:
					self.turn2 = 0
					self.turn3 = 1
					self.s2 = rospy.get_time()
					print('left2')
					self.twist.linear.x = 0.22
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(1.761)
					self.twist.linear.x = 0.2
					self.twist.angular.z = 1.3
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(2.4)
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
				elif self.turn3 == 1 and t2 - self.s2 > 7:
					self.turn3 = 0
					self.s3 = rospy.get_time()
					print('left3')
					self.twist.linear.x = 0.22
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(1.425)
					self.twist.linear.x = 0.2
					self.twist.angular.z = 0.95
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(2.9)
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)

			if (M1['m00'] > 0 or M2['m00'] > 0):
				if M1['m00'] == 0:
					cx1 = 200
					cy1 = 300  # complete left line (circle)
					cx2 = int(M2['m10'] / M2['m00'])
					cy2 = int(M2['m01'] / M2['m00'])
					fpt_x = (cx1 + cx2) / 2
					fpt_y = (cy1 + cy2) / 2
				elif M2['m00'] == 0:
					cx1 = int(M1['m10'] / M1['m00'])
					cy1 = int(M1['m01'] / M1['m00'])
					cx2 = 800
					cy2 = 300  # complete right line (circle)
					fpt_x = (cx1 + cx2) / 2
					fpt_y = (cy1 + cy2) / 2
				else:
					cx1 = int(M1['m10'] / M1['m00'])
					cy1 = int(M1['m01'] / M1['m00'])
					cx2 = int(M2['m10'] / M2['m00'])
					cy2 = int(M2['m01'] / M2['m00'])
					fpt_x = (cx1 + cx2) / 2
					fpt_y = (cy1 + cy2) / 2

				cv2.circle(BEV, (cx1, cy1), 10, (100, 255, 255), -1)
				cv2.circle(BEV, (cx2, cy2), 10, (100, 255, 255), -1)
				cv2.circle(BEV, (fpt_x, fpt_y), 10, (255, 100, 100), -1)

				err = 10 + w / 2 - fpt_x  # linear control
				alpha = -np.arctan2(fpt_x - w / 2, h - fpt_y)  # angular control

				self.twist.linear.x = 0.22
				# self.twist.angular.z = 2*alpha
				self.twist.angular.z = err * 0.01

				self.cmd_vel_pub.publish(self.twist)

			thistime = rospy.get_time()
			# avoid continuous turning
			if thistime - self.s3 >= 3 and self.right == 0 and M4['m00'] == 0 and self.finish == 0:
				self.right = 1
			if self.right == 1 and M4['m00'] > 0:
				print("right0")
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.8)
				self.twist.linear.x = 0.0
				self.twist.angular.z = -1.5
				self.cmd_vel_pub.publish(self.twist)
				self.right = 0
				self.finish = 1
				rospy.sleep(1.05)
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)	
			if self.finish == 1 and M0['m00'] == 0:
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.5)
				self.beginFollow = 0
			cv2.imshow("BEV", BEV)
			cv2.waitKey(1)

class navigator:
	def __init__(self):
		self.goalPoints = [ 
			# position(x,y,z) and pose(quaternion)
			# In this lab, x is from bottom to top and y is from right to left
			[(4.125, -0.05, 0.0), (0.0, 0.0, 0.737849902397, 0.67)], # p2  4964829848
			[(4.4, 3.8, 0.0), (0.0, 0.0, 0.025369587372, 0.999678140221)], # p3
			[(0.26703, 4.1, 0.0), (0.0, 0.0, 0.99998940623, 0.00460298028122)], # p4
			[(0.030, 0.00, 0.0), (0.0, 0.0, 0.73358, 0.719)], # p1
			[(0.735, 0.7, 0.0), (0.0, 0.0, 0.0456504303085, 1)], # lane_follow starting point
			[(1.66254192403, 0.647986888368, 0.0), (0.0, 0.0, 0.999174038183, 0.0406354699776)] # cross
		]	

	def init_send(self,pose):
		init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, latch=True, queue_size=1)
		init_msg = PoseWithCovarianceStamped()
		init_msg.header.frame_id = 'map'
		init_msg.pose.pose.position.x = pose[0][0]
		init_msg.pose.pose.position.y = pose[0][1]
		init_msg.pose.pose.position.z = pose[0][2]
		init_msg.pose.pose.orientation.x = pose[1][0]
		init_msg.pose.pose.orientation.y = pose[1][1]
		init_msg.pose.pose.orientation.z = pose[1][2]
		init_msg.pose.pose.orientation.w = pose[1][3]
		init_pub.publish(init_msg)

	def goal_send(self,pose):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = pose[0][0]
		goal.target_pose.pose.position.y = pose[0][1]
		goal.target_pose.pose.position.z = pose[0][2]
		goal.target_pose.pose.orientation.x = pose[1][0]
		goal.target_pose.pose.orientation.y = pose[1][1]
		goal.target_pose.pose.orientation.z = pose[1][2]
		goal.target_pose.pose.orientation.w = pose[1][3]
		return goal

	def navigation(self,num):
		client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
		client.wait_for_server(rospy.Duration(3))  # wait until server is initialized 

		goal = self.goal_send(self.goalPoints[num])
		client.send_goal(goal)
		client.wait_for_result(rospy.Duration(120))  # wait until the goal point is reached
		print(num)


if __name__=="__main__":
	rospy.init_node('navigation')
	
	n = navigator()
	c_rate = rospy.Rate(30)
	n.init_send(n.goalPoints[3])  # initial pose
	#navigation(1)
	for i in range(4):
		follower = Follower()
		rospy.sleep(1)  # wait for 1 second to build up related subscriber/publisher
		n.navigation(4)  # go to the starting point of the racetrack
		follower.s0 = rospy.get_time()
		follower.s1 = rospy.get_time()
		while follower.beginFollow == 1:
			follower.run()
			c_rate.sleep()
		n.navigation(0)  # go to point2
		follower.s1 = rospy.get_time()
		follower.beginFollow = 1  # restart lane following
		while follower.beginFollow == 1:
			follower.run()
			c_rate.sleep()
		print("point1")
		n.navigation(3)  # go to point1, 1 lap is finished
