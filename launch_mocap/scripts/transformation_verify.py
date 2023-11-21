#!/usr/bin/env python3

#Test Script to verify the transformation between the Qualisys and the MIR
#!!!Work in progress

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import tf2_geometry_msgs
import math

class verify:
    
    # topic1: Topic for the Qualisys data
    # topic2: Topic for the MIR data
    topic1_data = None
    topic2_data = None
    
    def main(self):
        topic1_map = None
        # Topic 2 doesn't need tranformation because it's already in the map frame
        topic2_map = self.topic2_data.position
                
        try:
            trans1 = self.tf_buffer.lookup_transform('map', 'mocap', rospy.Time())
            rospy.loginfo("Transformation found %s", trans1.transform.translation.x)
            topic1_map = tf2_geometry_msgs.do_transform_pose(self.topic1_data, trans1)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("No transformation found")
        
        rospy.loginfo("Qualisys x: %s, y: %s", topic1_map.pose.position.x, topic1_map.pose.position.y)
        rospy.loginfo("MIR x: %s, y: %s", topic2_map.x, topic2_map.y)
        
        if topic1_map is not None:
            diff = math.sqrt((topic2_map.x - (-1)*topic1_map.pose.position.x)**2 + (topic2_map.y - (-1)*topic1_map.pose.position.y)**2)
            
            rospy.loginfo("Difference: %s", diff)
            
            
            
            
    
    def __init__(self, *args):
        rospy.init_node('transformation_verify', anonymous=True)
        rospy.loginfo("Initializing node")
        rospy.Subscriber("/qualisys/mur620b/pose", PoseStamped, self.callback_1)
        rospy.Subscriber("/mur620b/robot_pose", Pose, self.callback_2)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf = tf2_ros.TransformListener(self.tf_buffer)
        
        self.rate = rospy.Rate(10)
        rospy.spin()
        
    def callback_1(self, data):
        self.topic1_data = data

    def callback_2(self, data):
        self.topic2_data = data
        if self.topic1_data is not None:
            self.main()
            
if __name__ == '__main__':
    Node = verify()
