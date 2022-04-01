#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point32, Twist
from std_msgs.msg import Header
from math import pi, cos, sin, radians, tan, degrees
import numpy as np


angle = 0
speed = 0

def cmd_vel_callback(data):
    global angle
    global speed
    #print(data.angular.z)
    angle = -radians(data.angular.z)
    speed = data.linear.x



rospy.init_node('trajectory', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
polygon_pub = rospy.Publisher("trajectory", PolygonStamped, queue_size=50)


rate = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    p = PolygonStamped()
    header = Header()
    header.frame_id = "mdynamix"
    header.stamp = rospy.Time.now()
    p.header = header

    vehicule_length = 36.5/100
    length = 2
    offset = 32/100
    largeur = 17/100    



    if speed > 0:
        if angle < radians(0.5) and angle > -radians(0.5) : #Dans ce cas, display un rectangle.
            p.polygon.points = [Point32(x=-largeur, y=0+offset, z=0),
                                Point32(x=largeur, y=0+offset, z=0),
                                Point32(x=largeur, y=length+offset, z=0),
                                Point32(x=-largeur, y=length+offset, z=0)]


        else : 
            rayon = vehicule_length/tan(angle)
            angle_traj = length/rayon
            pas = 1e-3

            if angle > 0:
                for theta in np.arange(0, angle_traj, pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = (rayon+largeur) * sin(theta)+offset
                    p.polygon.points.append(Point32(x=x, y=y))

                for theta in np.arange(angle_traj, 0, -pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = (rayon-largeur) * sin(theta)+offset
                    p.polygon.points.append(Point32(x=x, y=y))

            else :
                for theta in np.arange(0, angle_traj, -pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = (rayon+largeur) * sin(theta)+offset
                    p.polygon.points.append(Point32(x=x, y=y))

                for theta in np.arange(angle_traj, 0, pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = (rayon-largeur) * sin(theta)+offset
                    p.polygon.points.append(Point32(x=x, y=y))


    elif speed < 0 :
        if angle < radians(0.5) and angle > -radians(0.5) : #Dans ce cas, display un rectangle.
            p.polygon.points = [Point32(x=-largeur, y=0-offset, z=0),
                                Point32(x=largeur, y=0-offset, z=0),
                                Point32(x=largeur, y=-length-offset, z=0),
                                Point32(x=-largeur, y=-length-offset, z=0)]


        else : 
            rayon = vehicule_length/tan(angle)
            angle_traj = length/rayon
            pas = 1e-3

            if angle > 0:
                for theta in np.arange(0, angle_traj, pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = -((rayon+largeur) * sin(theta)+offset)
                    p.polygon.points.append(Point32(x=x, y=y))

                for theta in np.arange(angle_traj, 0, -pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = -((rayon-largeur) * sin(theta)+offset)
                    p.polygon.points.append(Point32(x=x, y=y))

            else :
                for theta in np.arange(0, angle_traj, -pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = -((rayon+largeur) * sin(theta)+offset)
                    p.polygon.points.append(Point32(x=x, y=y))

                for theta in np.arange(angle_traj, 0, pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = -((rayon-largeur) * sin(theta)+offset)
                    p.polygon.points.append(Point32(x=x, y=y))


    elif speed == 0 :
        p.polygon.points = [Point32(x=-34/200, y=-65/200, z=0),
                        Point32(x=34/200, y=-65/200, z=0),
                        Point32(x=34/200, y=65/200, z=0),
                        Point32(x=-34/200, y=65/200, z=0)]





    polygon_pub.publish(p)
    rate.sleep()

