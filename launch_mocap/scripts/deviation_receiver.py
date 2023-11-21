#!/usr/bin/env python3

"""
This code performs a rotation of the robot ariund its z axis and calculates the radius of the circle.
The x and y coordinates of the robot are plotted in real time.
The radius of the circle is calculated by taking the average of the distance between the center of the circle and the points on the circle.
"""

from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from tf import transformations
import math
from matplotlib import pyplot as plt

class node():
    # Initialize ros qariables
    point_list = []
    x_data = []
    y_data = []
    start = False
    stop = False
    starting_pose = Pose()
    current_pose = PoseStamped()
    target_orientation = None
    
    # Initialize the animation variables
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'o-')

    def config(self):
        self.robot_pose_topic_mocap = "/qualisys/mur620a/pose"
        self.cmd_vel_topic = "/cmd_vel"

    # Constructor which initializes the node and the publisher, the subscriber and the animation
    def __init__(self):
        self.config()
        rospy.init_node('data_receiver', anonymous=True)
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.robot_pose_topic_mocap, PoseStamped, self.callback)
        rospy.spin()

    def run(self):
        # Set the target orientation to 1.9 * pi radians because a whole rotation can cause problems with the calculation of the radius 
        if self.target_orientation is None:
            rot = transformations.quaternion_about_axis(1.9 * math.pi, (0,0,1))
            q = [self.starting_pose.orientation.x, self.starting_pose.orientation.y, self.starting_pose.orientation.z, self.starting_pose.orientation.w]
            q_target = transformations.quaternion_multiply(q, rot)
            self.target_orientation = transformations.euler_from_quaternion(q_target)
            rospy.loginfo("Target orientation: %s", self.target_orientation)

        # Rotate the robot around its z axis until it reaches the target orientation
        # self.start is True when the robot has received its first pose
        # self.stop is set to True when the robot has reached the target orientation
        if self.start is True and hasattr(self.current_pose, 'orientation') :
            current_orientation = transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            if abs(current_orientation[2] - self.target_orientation[2]) > 0.1:
                twist = Twist()
                twist.angular.z = 0.1
                self.pub.publish(twist)
                rospy.loginfo("%s %s", self.current_pose.position.x, self.current_pose.position.y)
                self.x_data.append(self.current_pose.position.x)
                self.y_data.append(self.current_pose.position.y)
                self.update_plot(None)
            else:             # Stop the robot and start to calculate the radius of the circle
                twist = Twist()
                twist.linear.x = 0.0
                self.pub.publish(twist)
                self.stop = True
                self.circle(self.point_list)

    def update_plot(self, frame):
        self.line.set_data(self.x_data, self.y_data)
        self.line.set_markersize(0.1)
        return self.line,

    def callback(self, data):
        if self.start is False:
            self.starting_pose = data.pose
            self.start = True
        elif self.stop is False:
            self.current_pose = data.pose
            self.point_list.append(data.pose)
        self.run()

    # Calculate the radius of the circle
    # The center of the circle is the average of the x, y and z coordinates of the points on the circle
    # The radius of the circle is the average of the distance between the center of the circle and the points on the circle
    def circle(self, points):
        x, y, z = 0, 0, 0
        for point in points:
            x += point.position.x
            y += point.position.y
            z += point.position.z
        x /= len(points)
        y /= len(points)
        z /= len(points)
        rospy.loginfo("Center of circle: (%s, %s, %s)", x, y, z)
        radius = []
        for point in points:
            radius.append(math.sqrt((point.position.x - x)**2 + (point.position.y - y)**2 + (point.position.z - z)**2))
        rospy.loginfo("Radius of circle: %s", sum(radius)/len(radius))
        self.ax.autoscale(enable=True, axis='both', tight=True)
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=10, blit=True, cache_frame_data=False)
        plt.show()
        rospy.signal_shutdown("Shutting down")



if __name__ == '__main__':
    Node = node()
    #Node.run()