#!/usr/bin/env python3

# this code is used to calibrate the orientation between the mocap and the robot

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from copy import deepcopy
from tf import transformations
import math

class AngleCalibration():
    
    def __init__(self) -> None:
        self.config()
        rospy.Subscriber(self.robot_pose_topic_mocap, PoseStamped, self.mocap_pose_callback)
        self.twist_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        
        
    def config(self):
        self.robot_pose_topic_mocap = "/qualisys/mur620a/pose"
        self.cmd_vel_topic = "/mur620a/cmd_vel"
        self.duration = 10
        self.velocity = 0.1
    
    def run(self):
        # store inital position
        self.initial_position = deepcopy(self.mocap_pose)
        
        # get current time 
        self.start_time = rospy.Time.now()
        
        # drive the robot forwards for 10 seconds, if position of the robot only changes in x direction, the orientation is correct
        twist = Twist()
        twist.linear.x = self.velocity
        while rospy.Time.now() - self.start_time < rospy.Duration(self.duration):
            self.twist_pub.publish(twist)
        
        self.final_position = deepcopy(self.mocap_pose)
        
        # compute the initial and the orientation in which the robot acutally moved
        initial_phi = transformations.euler_from_quaternion([self.initial_position.pose.orientation.x, self.initial_position.pose.orientation.y, self.initial_position.pose.orientation.z, self.initial_position.pose.orientation.w])[2]
        actual_phi = math.atan2(self.final_position.pose.position.y - self.initial_position.pose.position.y, self.final_position.pose.position.x - self.initial_position.pose.position.x)
        
        # compute the difference between the two angles
        self.angle_difference = actual_phi - initial_phi
        
        # print the angle difference
        print("initial phi: ", initial_phi)
        print("actual phi: ", actual_phi)
        print("difference: ", self.angle_difference)
        
        
        
        
        
    def mocap_pose_callback(self, mocap_pose = PoseStamped()):
        self.mocap_pose = mocap_pose
     
        
    
if __name__ == "__main__":
    rospy.init_node('angle_calibration', anonymous=True)
    angle_calibration = AngleCalibration()
    angle_calibration.run()
    rospy.spin()