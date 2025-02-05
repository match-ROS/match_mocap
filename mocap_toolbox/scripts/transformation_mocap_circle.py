#!/usr/bin/env python3

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
import tf
import tf2_geometry_msgs.tf2_geometry_msgs

# Python
import numpy as np
import math
import json

# Scipy
import statistics as st
from scipy.optimize import least_squares


class Q2M_Transformer:

    # Function to calculate the intersection of two circles
    def get_intersections(self, x0, y0, r0, x1, y1, r1):
        d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
        
        if d > r0 + r1 :
            return None

        if d < abs(r0-r1):
            return None

        if d == 0 and r0 == r1:
            return None

        else:
            a=(r0**2-r1**2+d**2)/(2*d)
            h=math.sqrt(r0**2-a**2)
            x2=x0+a*(x1-x0)/d   
            y2=y0+a*(y1-y0)/d   
            x3=x2+h*(y1-y0)/d     
            y3=y2-h*(x1-x0)/d 

            x4=x2-h*(y1-y0)/d
            y4=y2+h*(x1-x0)/d
            
            return (x3, y3, x4, y4)

    # Functions which are used for least squares optimization 
    # See Readme for more information
    def func_x(self, phi, ks_base_x, x1, y1, sol):
        return ks_base_x + x1 * np.cos(phi) - y1 * np.sin(phi) - sol

    def func_y(self, phi, ks_base_y, x1, y1, sol):
        return ks_base_y + x1 * np.sin(phi) + y1 * np.cos(phi) - sol

    # Main function
    # ps_q: PoseStamped from Qualisys
    # p_m: Pose from MIR
    # Returns: two lists with the results for phi and the base of the Qualisys coordinate system in mir coordinates
    def main(self, ps_q, p_m):
        points_qualisys = []
        points_mir = []
        # [ q_x, q_y, r]
        circles = []

        # Transform coordinates from PoseStamped and Pose to list
        for point_q in ps_q:
            x = float(point_q.pose.position.x)
            y = float(point_q.pose.position.y)
            points_qualisys.append([x, y])

        for point_m in p_m:
            x = float(point_m.position.x)
            y = float(point_m.position.y)
            points_mir.append([x, y])

        # ! Now the calculation of the base starts
        # Calculate the radius of the circles with the Qualisys coordinates
        i = 0
        while i < len(points_qualisys):
            qualisys_x = points_qualisys[i][0]
            qualisys_y = points_qualisys[i][1]
            radius = math.sqrt( math.pow(qualisys_x, 2) + math.pow(qualisys_y, 2))
            
            mir_x = points_mir[i][0]
            mir_y = points_mir[i][1]
            
            circles.append([mir_x, mir_y, radius])

            i = i + 1

        # intersection_1 and intersection_2 are lists with the intersection points of the circles
        intersection_1 = []
        intersection_2 = []
        KS_Base = []
        
        # Calculate the intersection points of the circles and checks if there are two intersections
        i1 = self.get_intersections(circles[0][0], circles[0][1], circles[0][2], circles[1][0], circles[1][1], circles[1][2])
        if i1 is not None:
            intersection_1.extend(i1)
        else:
            rospy.loginfo("No intersecting circles")
            return None, None, None
        i2 = self.get_intersections(circles[1][0], circles[1][1], circles[1][2], circles[2][0], circles[2][1], circles[2][2])
        if i2 is not None:
            intersection_2.extend(i2)
        else:
            rospy.loginfo("No intersecting circles")
            return None, None, None

        # Trys to find the two intersection points which are mostly the same
        for point1 in intersection_1:
            for point2 in intersection_2:
                if (point1 - point2) < 1 and (point1 - point2) > -1 and len(KS_Base) < 2:
                    KS_Base.append(np.mean([point1, point2]))

        if len(KS_Base) != 2:
            rospy.loginfo("No Base")
            return None, None, None
        
        # !Now the calculation of the angle phi starts
        result_x = []
        result_y = []
        i = 0
        # For each point calculate the angle phi
        while i < len(points_qualisys):
            qualisys_x = points_qualisys[i][0]
            qualisys_y = points_qualisys[i][1]
            mir_x = points_mir[i][0]
            mir_y = points_mir[i][1]
            # Calculate the angle phi with least squares optimization
            # The function func_x and func_y are used for the optimization
            # See Readme for more information
            result1 = least_squares(self.func_x, 1, args=(KS_Base[0], qualisys_x, qualisys_y, mir_x))
            result2 = least_squares(self.func_y, 1, args=(KS_Base[1], qualisys_x, qualisys_y, mir_y))
            # Check if the results are valid
            if len(result1.x) != 0:
                result_x.append(result1.x[0])
            else:
                rospy.loginfo("No result for x-coordinates")
                return None, None, None
            if len(result2.x) != 0:
                result_y.append(result2.x[0])
            else:
                rospy.loginfo("No result for y-coordinates")
                return None, None, None
            i = i +1 
        # Return the results
        return result_x, result_y, KS_Base
    
# Main class for the node
class Main():

    # topic1: Topic for the Qualisys data
    # topic2: Topic for the MIR data
    topic1_data = None
    topic2_data = None

    b = True
    rate = None

    point_q_arr = []
    point_m_arr = []

    xy_list = []
    y_list = []
    ksbase_list = []
    list_transformations = []
    offset_list = []

    i = 0

    def transform_point(self, point_qualisys, point_mir, trans_x, trans_y, trans_c):
        # Create a transformation from the results of the calculation
        transformation = TransformStamped()
        transformation.header.stamp = rospy.Time.now()
        transformation.header.frame_id = "map"
        transformation.child_frame_id = str(len(self.list_transformations) + 1)
        transformation.transform.translation.x = trans_x
        transformation.transform.translation.y = trans_y
        transformation.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, trans_c)
        transformation.transform.rotation.x = q[0]
        transformation.transform.rotation.y = q[1]
        transformation.transform.rotation.z = q[2]
        transformation.transform.rotation.w = q[3]

        # Transform the Qualisys point with the calculated transformation
        point_transformed = tf2_geometry_msgs.do_transform_pose(point_qualisys, transformation)

        # Calculate the offset between the transformed Qualisys point and the MIR point
        x_offset = point_transformed.pose.position.x - point_mir.position.x
        y_offset = point_transformed.pose.position.y - point_mir.position.y
        yaw_offset = point_transformed.pose.orientation.z - point_mir.orientation.z
        good = False
        # Check if the offset is small enough
        if ( x_offset < 0.01 and x_offset > -0.01):
            if ( y_offset < 0.01 and y_offset > -0.01):
                if ( yaw_offset < 0.01 and yaw_offset > -0.01) and trans_c > 0 and trans_c < math.pi:
                    rospy.loginfo("Found good transformation! x:%s y:%s phi:%s", trans_x, trans_y, trans_c)
                    rospy.loginfo("Offset: x:%s y:%s phi:%s", x_offset, y_offset, yaw_offset)
                    self.list_transformations.append(transformation)
                    self.offset_list.append([x_offset, y_offset, yaw_offset])
                    rospy.loginfo("Current good transformations: %s", len(self.list_transformations))
                    b = True
        if good is False:
            rospy.loginfo("Transformation was bad! %s %s %s %s", x_offset, y_offset, yaw_offset, str(len(self.list_transformations)))
        # If we have x good transformations, calculate the median of the results and stop the node, x is configured in the config file
        if len(self.list_transformations) == self.good_tranformations:
            x_list = []
            y_list = []
            c_list = []
            for trans in self.list_transformations:
                x_list.append(trans.transform.translation.x)
                y_list.append(trans.transform.translation.y)
                # Calculate the euler angles from the quaternion
                explicit_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                c_list.append(yaw)
            i = 0

            if self.print_transformation is True:
                while i < len(self.list_transformations):
                    rospy.loginfo("Transformation %s", i)
                    rospy.loginfo("x: %s", x_list[i])
                    rospy.loginfo("y: %s", y_list[i])
                    rospy.loginfo("phi: %s", c_list[i])
                    i = i + 1
            
            # Find the transformation with the lowest offset
            lowest_offset_index = min(range(len(self.offset_list)), key=lambda i: abs(self.offset_list[i][0]) + abs(self.offset_list[i][1]) + abs(self.offset_list[i][2]))
            
            rospy.loginfo("-!---------------------------------!-")
            rospy.loginfo("Reached %s good transformations!")
            rospy.loginfo("")
            rospy.loginfo("Median: ")
            rospy.loginfo("x: %s", st.median(x_list))
            rospy.loginfo("y: %s", st.median(y_list))
            rospy.loginfo("phi: %s rad", st.median(c_list))
            rospy.loginfo("")
            rospy.loginfo("Lowest offset: ")
            rospy.loginfo("x: %s", x_list[lowest_offset_index])
            rospy.loginfo("y: %s", y_list[lowest_offset_index])
            rospy.loginfo("phi: %s rad", c_list[lowest_offset_index])
            rospy.loginfo("-!---------------------------------!-")
            self.b = False
            trans_median = TransformStamped()
            trans_median.header.stamp = rospy.Time.now()
            trans_median.header.frame_id = "map"
            trans_median.child_frame_id = "mocap"
            if(self.use_median is False):
                trans_median.transform.translation.x = x_list[lowest_offset_index]
                trans_median.transform.translation.y = y_list[lowest_offset_index]
                q = tf.transformations.quaternion_from_euler(0, 0, c_list[lowest_offset_index])
            else:
                trans_median.transform.translation.x = st.median(x_list)
                trans_median.transform.translation.y = st.median(y_list)
                q = tf.transformations.quaternion_from_euler(0, 0, st.median(c_list))
            
            trans_median.transform.translation.z = 0.0
            trans_median.transform.rotation.x = q[0]
            trans_median.transform.rotation.y = q[1]
            trans_median.transform.rotation.z = q[2]
            trans_median.transform.rotation.w = q[3]
            if self.publish_transformation is True:
                self.broadcaster(trans_median)
            else:
                rospy.signal_shutdown("END!")

            

    def broadcaster(self, trans):
        while rospy.is_shutdown() is False:
            bc = tf.TransformBroadcaster()
            translation = (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            )
            rotation = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            bc.sendTransform(translation, rotation, rospy.Time.now(), "mocap", "map")
            self.rate.sleep()

    # Function which handles the data from the topics
    def process_data(self, topic1_data, topic2_data):
        # Check if the data is not None and we still need to find the transformation
        if topic1_data is not None and topic2_data is not None and self.b is True:
            self.point_q_arr.append(topic1_data)
            self.point_m_arr.append(topic2_data)
            self.i = self.i + 1
            # If we have 3 points, we can calculate the transformation
            if self.i == 3:
                # Calculate the transformation with the Q2M_Transformer class
                q2m = Q2M_Transformer()
                rs_x, rs_y, ksb = q2m.main(self.point_q_arr, self.point_m_arr)
                # If the transformation is correct, we add it to the list
                if rs_x is not None and rs_y is not None and ksb is not None:
                    if len(rs_x) != 0 and len(rs_y) != 0:
                        self.ksbase_list.append(ksb)
                        for s in rs_x:
                            self.xy_list.append(s)
                        for s in rs_y:
                            self.xy_list.append(s)
                # Clear the lists for the next 3 points
                self.point_q_arr.clear()
                self.point_m_arr.clear()
                # Check the transformation with the current received points
                for s in self.xy_list:
                    self.transform_point(topic1_data, topic2_data, self.ksbase_list[0][0], self.ksbase_list[0][1], s)
                # Clear the lists for the next 3 points
                self.ksbase_list.clear()
                self.xy_list.clear()
                self.i = 0

    # Callback functions for the topics
    def callback_1(self, data):
        self.topic1_data = data

    def callback_2(self, data):
        self.topic2_data = data
        # Because the topics are not synchronized, we use the slowest topic as a trigger for the procedure
        if self.i < 3:
            self.process_data(self.topic1_data, self.topic2_data)
            
    def load_config(self, path):
        with open(path, 'r') as config:#
            return json.load(config)

    # init function
    # will activate the node and subscribe to the topics
    def __init__(self):
        rospy.init_node('transformation_mocap', anonymous=True)
        rospy.loginfo("Start")
        self.publish_transformation = rospy.get_param("~general_parameters/publish_transformation", False)
        self.use_median = rospy.get_param("~general_parameters/use_median", False)
        self.good_tranformations = rospy.get_param("~general_parameters/good_transformations", 50)
        self.print_transformation = rospy.get_param("~general_parameters/print_all_transformations", False)
        self.qualisys_topic = rospy.get_param("~general_parameters/qualisys_topic", "/qualisys/mur620d/pose")
        self.mir_topic = rospy.get_param("~general_parameters/mir_topic", "/mur620d/robot_pose")
        rospy.Subscriber(self.qualisys_topic, PoseStamped, self.callback_1)
        rospy.Subscriber(self.mir_topic, Pose, self.callback_2)
        self.rate = rospy.Rate(10)
        rospy.spin()
    
if __name__ == '__main__':
    m = Main()
    m.__init__()