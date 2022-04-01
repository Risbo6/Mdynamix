#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Header
from math import pi, cos, sin, radians
import numpy as np

rospy.init_node('polygon', anonymous=True)

polygon_pub = rospy.Publisher("polygon", PolygonStamped, queue_size=50)


rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    p = PolygonStamped()
    header = Header()
    header.frame_id = "mdynamix"
    header.stamp = rospy.Time.now()
    p.header = header
    
    p.polygon.points = [Point32(x=-34/200, y=-65/200, z=0),
                        Point32(x=34/200, y=-65/200, z=0),
                        Point32(x=34/200, y=50/200, z=0),
                        Point32(x=34/200, y=65/200, z=0),
                        Point32(x=-34/200, y=65/200, z=0),
                        Point32(x=-34/200, y=50/200, z=0)]


    



    polygon_pub.publish(p)
    rate.sleep()
