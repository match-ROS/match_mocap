#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from threading import Lock
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from std_srvs.srv import Empty, EmptyResponse

class TransformationCalculator:
    def __init__(self):
        rospy.init_node('transformation_calculator')

        # Parameter
        robot_pose_map_topic = rospy.get_param('~robot_pose_map_topic', '/mur620d/robot_pose')  # Topic für die Roboterpose im Map-KS
        robot_pose_mocap_topic = rospy.get_param('~robot_pose_mocap_topic', '/qualisys/mur620d/pose')  # Topic für die Roboterpose im MoCap-KS
        odom_topic = rospy.get_param('~odom_topic', '/mur620d/odom')  # Topic für die Odometrie
        self.minimum_pose_pairs = rospy.get_param('~minimum_pose_pairs', 3)  # Mindestanzahl der benötigten Pose-Paare
        self.velocity_threshold_linear = rospy.get_param('~velocity_threshold_linear', 0.01)  # Schwelle für lineare Geschwindigkeit (m/s)
        self.velocity_threshold_angular = rospy.get_param('~velocity_threshold_angular', 0.01)  # Schwelle für Winkelgeschwindigkeit (rad/s)
        self.lock = Lock()


        # Listen für gemittelte Pose-Paare
        self.pose_pairs = []

        # Temporäre Listen für Posen während des Stillstands
        self.temp_map_poses = []
        self.temp_mocap_poses = []

        # Variablen für die Geschwindigkeiten des Roboters
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        # Zustandsvariable, ob der Roboter gerade stillsteht
        self.robot_stationary = False

        # Subscribers
        self.map_pose_sub = rospy.Subscriber(robot_pose_map_topic, Pose, self.map_pose_callback)
        self.mocap_pose_sub = rospy.Subscriber(robot_pose_mocap_topic, PoseStamped, self.mocap_pose_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Service zum Auslösen der Transformation
        self.transform_service = rospy.Service('~compute_transformation', Empty, self.compute_transformation_service)

        rospy.loginfo("Transformation Calculator gestartet.")
        rospy.loginfo("Bewegen Sie den Roboter zu verschiedenen Positionen und halten Sie ihn an, um Daten zu sammeln.")
        rospy.loginfo("Rufen Sie den Service '/transformation_calculator/compute_transformation' auf, um die Transformation zu berechnen.")

    def odom_callback(self, msg):
        # Aktualisiert die aktuellen Geschwindigkeiten des Roboters
        self.current_linear_velocity = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.current_angular_velocity = abs(msg.twist.twist.angular.z)
        # Überprüft, ob der Roboter jetzt stillsteht
        stationary = self.is_robot_stationary()
        if stationary and not self.robot_stationary:
            rospy.loginfo("Roboter ist jetzt im Stillstand. Beginne mit Datensammlung.")
            self.robot_stationary = True
        elif not stationary and self.robot_stationary:
            rospy.loginfo("Roboter hat sich bewegt. Berechne gemittelte Pose.")
            self.robot_stationary = False
            self.compute_average_pose()

    def is_robot_stationary(self):
        # Überprüft, ob der Roboter stillsteht
        return (self.current_linear_velocity < self.velocity_threshold_linear and
                self.current_angular_velocity < self.velocity_threshold_angular)

    def map_pose_callback(self, msg):
        if self.robot_stationary:
            with self.lock:
                self.temp_map_poses.append(msg)

    def mocap_pose_callback(self, msg):
        if self.robot_stationary:
            with self.lock:
                self.temp_mocap_poses.append(msg.pose)

    def compute_average_pose(self):
        with self.lock:
            rospy.loginfo(f"Gesammelte Map Poses: {len(self.temp_map_poses)}")
            rospy.loginfo(f"Gesammelte Mocap Poses: {len(self.temp_mocap_poses)}")
            if len(self.temp_map_poses) > 0 and len(self.temp_mocap_poses) > 0:
                # Mittelung der Map-Posen
                avg_map_pose = self.average_poses(self.temp_map_poses)

                # Mittelung der MoCap-Posen
                avg_mocap_pose = self.average_poses(self.temp_mocap_poses)

                # Speichern des Pose-Paares
                self.pose_pairs.append((avg_map_pose, avg_mocap_pose))
                rospy.loginfo(f"Gesammelte Pose-Paare: {len(self.pose_pairs)}")

                # Leeren der temporären Listen
                self.temp_map_poses = []
                self.temp_mocap_poses = []

                # Überprüfen, ob die Mindestanzahl erreicht ist
                if len(self.pose_pairs) >= self.minimum_pose_pairs:
                    rospy.loginfo("Mindestanzahl an Pose-Paaren erreicht. Sie können nun den Service aufrufen, um die Transformation zu berechnen.")
            else:
                rospy.logwarn("Nicht genügend Posen für Mittelung während des Stillstands.")

    def average_poses(self, poses):
        # Mittelung der Positionen
        positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in poses])
        avg_position = np.mean(positions, axis=0)

        # Mittelung der Orientierungen (Quaternionen)
        quaternions = np.array([[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] for pose in poses])
        avg_quaternion = self.average_quaternions(quaternions)

        # Erstellen der gemittelten Pose
        avg_pose = PoseStamped().pose
        avg_pose.position.x = avg_position[0]
        avg_pose.position.y = avg_position[1]
        avg_pose.position.z = avg_position[2]
        avg_pose.orientation.x = avg_quaternion[0]
        avg_pose.orientation.y = avg_quaternion[1]
        avg_pose.orientation.z = avg_quaternion[2]
        avg_pose.orientation.w = avg_quaternion[3]

        return avg_pose

    def average_quaternions(self, quaternions):
        # Gewichtete Mittelung der Quaternionen
        # Hier wird die Methode von Markley verwendet
        A = np.zeros((4, 4))
        for q in quaternions:
            q = q / np.linalg.norm(q)  # Normierung
            A += np.outer(q, q)
        eigenvalues, eigenvectors = np.linalg.eigh(A)
        avg_quaternion = eigenvectors[:, np.argmax(eigenvalues)]
        return avg_quaternion

    def compute_transformation_service(self, request):
        with self.lock:
            if len(self.pose_pairs) < self.minimum_pose_pairs:
                rospy.logerr("Nicht genügend Pose-Paare für die Berechnung der Transformation.")
                return EmptyResponse()

            # Berechnung der Transformation basierend auf den gesammelten Pose-Paaren
            R, t = self.compute_transformation(self.pose_pairs)

            rospy.loginfo("Transformation vom MoCap-KS zum Map-KS:")
            rospy.loginfo("Rotationsmatrix:")
            rospy.loginfo(R)
            rospy.loginfo("Translationsvektor:")
            rospy.loginfo(t)

            # Transformationsmatrix erstellen
            transformation_matrix = np.identity(4)
            transformation_matrix[0:3, 0:3] = R
            transformation_matrix[0:3, 3] = t

            # Quaternion aus der Rotationsmatrix erhalten
            q = quaternion_from_matrix(transformation_matrix)

            # Transform veröffentlichen
            br = tf.TransformBroadcaster()
            rate = rospy.Rate(10)
            rospy.loginfo("Veröffentliche Transformation zwischen 'map' und 'mocap' Frames.")
            while not rospy.is_shutdown():
                br.sendTransform(t,
                                 q,
                                 rospy.Time.now(),
                                 "map",
                                 "mocap")
                rate.sleep()
        return EmptyResponse()

    def compute_transformation(self, pose_pairs):
        # Extrahiert Positionen
        map_positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose, _ in pose_pairs])
        mocap_positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for _, pose in pose_pairs])

        # Berechnet die Zentren
        centroid_map = np.mean(map_positions, axis=0)
        centroid_mocap = np.mean(mocap_positions, axis=0)

        # Zentriert die Positionen
        map_centered = map_positions - centroid_map
        mocap_centered = mocap_positions - centroid_mocap

        # Berechnet die Kovarianzmatrix
        H = np.dot(mocap_centered.T, map_centered)

        # Führt SVD durch
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # Korrigiert für Reflektion
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)

        # Berechnet die Translation
        t = centroid_map - np.dot(R, centroid_mocap)

        return R, t

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TransformationCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass