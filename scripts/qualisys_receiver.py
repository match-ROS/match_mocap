#!/usr/bin/env python

"""
Script to receive data from Qualisys motion capture system
and publish it to ROS topic.
First argument should be the id of the rigid body.
If no argument is given, all rigid bodies are printed.
"""

import rospy
import qtm
import asyncio
import xml.etree.ElementTree as ET
import sys
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class node:
    #Initialize the variables
    is_connected = False
    shutdown = False
    connection = None
    rigid_bodies = []
    
    #min and max id of the points of the rigid body
    min_id = 0
    max_id = 0
    points = []

    #Publisher for the data
    pub = rospy.Publisher('qualisys_data', PointStamped, queue_size=10)


    #Function to get the data from the Qualisys system 
    async def get_info(self, ip):

        #Callback function for the data stream      
        def on_packet(packet):
                #Get the position of the points with the ids
                info, markers = packet.get_3d_markers()
                #Start with the min_id of the rigid body given by the argument and publish the data
                i = self.min_id
                #Publish the data until the max_id is reached
                while i <= self.max_id:
                    #Get the position of the point with the id i
                    position = markers[i]
                    #Create a PointStamped message and publish it
                    ps = PointStamped()
                    ps.point.x = float(position.x)
                    ps.point.y = float(position.y)
                    ps.point.z = float(position.z)
                    ps.header.frame_id = str(i)
                    self.points.append(ps)
                    self.pub.publish(ps)
                    i = i+1


        #Startup procedure to get the rigid bodies and the ids of the points
        if self.is_connected is False:
            #Connect to the mocap system
            self.connection = await qtm.connect(ip)
            self.is_connected = True
            rospy.loginfo("%s", self.is_connected)

            #Get the rigid bodies from a xml string
            xml_string = await self.connection.get_parameters(parameters=["6d"])
            xml = ET.fromstring(xml_string)     
            #Add the rigid bodies to the list bodies   
            bodies = xml.findall('.//Body')

            #Next step is to get the ids of the points of the rigid bodies
            #Every point of a rigid body has an unique id
            #We need the min and max id of the points of the rigid body to publish the correct data 
            max = 0
            id = 0
            for body in bodies:
                name = body.find('Name').text
                #Get the number of points of the rigid body
                points = body.findall('.//Point')
                num_points = len(points)
                #Add the rigid body to the list rigid_bodies with the min and max id of the points
                self.rigid_bodies.append({'id': id, 'name': name, 'min_id': max, 'max_id': (num_points + max - 1)})
                #New max id is the old max id + the number of points of the rigid body
                max = max + num_points
                id += 1

            #If no argument is given, print the rigid bodies
            if len(sys.argv) == 1:
                rospy.loginfo("%s", self.rigid_bodies)
            
            #If an argument is given, set the min and max id of the points of the rigid body to publish
            else:
                id = int(sys.argv[1])
                body = self.rigid_bodies[id]
                self.min_id = body['min_id']
                self.max_id = body['max_id']
            #Tell asyncio that the startup procedure is finished
            await asyncio.sleep(0)

        #If the startup procedure is finished, start the data stream
        else:
            #Start the data stream with the components 3d (gets only the position of the points with the ids)
            #on_packet is the callback function if a packet is received
            await self.connection.stream_frames(components=["3d"], on_packet=on_packet)
            #Wait until the data stream is stopped
            await self.connection.stream_frames_stop()
            #Tell asyncio that the data stream is finished
            await asyncio.sleep(0)     
        
    #Main function
    async def run(self):
        #Initialize the node
        rospy.init_node('qualisys_receiver')
        #Set the rate to 10 Hz
        self.rate = rospy.Rate(10)
        #Start the data stream and the startup procedure
        while not rospy.is_shutdown():
            await self.get_info("192.168.254.1")
            #Sleep for 1 second after the data stream is finished
            rospy.sleep(1)

if __name__ == '__main__':
    node = node()
    #Start the node with asyncio because the qtm module is asynchronous
    loop = asyncio.run(node.run())

