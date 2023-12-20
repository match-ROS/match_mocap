#!/usr/bin/env python3

import rospy
# import tf2_ros
import tf
from geometry_msgs.msg import PoseStamped
from rospy import names

class MocapTransformerNode:
    def __init__(self):
        """Transforms mocap poses to map frame (from mocap/* to mocap_map/*)"""
        rospy.init_node('mocap_transformer_node', anonymous=True)

        #  Get params:
        self.target_frame = rospy.get_param("~target_frame", "map")

        self.tf_listener = tf.TransformListener()
        # Subscribe to PoseStamped topics under /mocap/*
        self.mocap_subscribers = {}
        self.mocap_publishers = {}

        # Find all available mocap topics dynamically
        self.discover_mocap_topics()

        # Define a callback function for each mocap topic
        for topic in self.mocap_topics:
            self.mocap_publishers[topic] = rospy.Publisher(
                f'/qualisys_{self.target_frame}/{topic}', PoseStamped, queue_size=10
            )
            
            self.mocap_subscribers[topic] = rospy.Subscriber(
                f'/qualisys/{topic}', PoseStamped, self.mocap_callback, callback_args=topic
            )

        rospy.spin()

    def discover_mocap_topics(self):
        # Use master API to get list of published topics
        try:
            all_topics_with_type = rospy.get_published_topics("/qualisys/")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Failed to get published topics: {e}")
            return

        # Extract topics under /mocap/* that are of type geometry_msgs/PoseStamped
        mocap_topics = [topic[0].replace("/qualisys","") for topic in all_topics_with_type if topic[1] == "geometry_msgs/PoseStamped"]

        if not mocap_topics:
            rospy.logwarn("No mocap topics found. Make sure mocap topics are being published.")
        else:
            rospy.loginfo(f"Discovered mocap topics: {mocap_topics}")
        
        self.mocap_topics = mocap_topics

    def mocap_callback(self, pose_stamped, topic):
        try:            
            # Transform the pose
            transformed_pose = self.tf_listener.transformPose(self.target_frame, pose_stamped)

            # Publish the transformed pose on /mocap_map/*
            self.mocap_publishers[topic].publish(transformed_pose)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(1, f"Transform lookup failed for {topic} ({pose_stamped.header.frame_id} to {self.target_frame}): {e}")

if __name__ == '__main__':
    try:
        mocap_transformer_node = MocapTransformerNode()
    except rospy.ROSInterruptException:
        pass
