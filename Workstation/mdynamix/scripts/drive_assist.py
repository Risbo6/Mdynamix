#!/usr/bin/env python3
from cmath import inf
from concurrent.futures import thread
from curses import KEY_PPAGE
from os import kill
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, radians, tan, atan2, degrees
import numpy as np
from numpy.linalg import norm
import matplotlib.path as mpltPath
import matplotlib.pyplot as plt
import threading
from simple_pid import PID

# PD tuning
Kp = 3
Kd = 0
setpoint = 5 # Distance à laquelle le robot va commencer à ralentir s'il y a un obstacle
max_speed = 20 # Maximum speed that the robot will reach
max_speed_allowed = 0
mutex = threading.Lock()
angle = 0 # Angle des roues
speed = 0 # Vitesse du véhicule avant drive assist
p_np = None # Polygon numpy qui represente la trajectoire
inf_ = 9999 # Presque inf. Utilisé pour représenter la distance du véhicule au non-obstacle

dist_obstacle_min = inf_ # Distance obstacle le plus proche dans la trajectoire.
obstacle_position = np.array([[2, 2], [0, 0.5]], ndmin=2)  # Liste des postions (x,y) des obstacles capturés par le lidar

# Calcul de la distance du véhicule à l'obstacle en passant par la trajectoire en arc.
def distance_obstacle_arc(rayon_f, offset_f, point_obstacle_f, angle_f, debug=False):
    #Center of circle
    circle_center_f = np.array([-rayon_f, offset_f], ndmin=2)

    #Calcul de l'angle entre l'horizontal et l'obstacle en degre
    x_obstacle_f = circle_center_f[0,0]-point_obstacle_f[0,0]
    y_obstacle_f = -circle_center_f[0,1]+point_obstacle_f[0,1]

    if angle_f > 0 :
        angle_obstacle_f = degrees(atan2(y_obstacle_f, -x_obstacle_f))
    else:
        angle_obstacle_f = degrees(atan2(y_obstacle_f, x_obstacle_f))

    angle_obstacle_f = angle_obstacle_f%360

    #Calcul de la distance entre l'obstacle et le vehicule (arc)
    rayon_obstacle_f = norm([x_obstacle_f, y_obstacle_f])
    dist_obstacle_f = rayon_obstacle_f*radians(angle_obstacle_f)

    if debug :
        print(f'Angle: {angle_obstacle_f}°, distance: {dist_obstacle_f} m')

    return dist_obstacle_f

def cmd_vel_callback(data):
    global angle
    global speed
    
    angle = -radians(data.angular.z)
    speed = data.linear.x


def lidar_callback(data):
    global obstacle_position
    global mutex
    mutex.acquire() # Capture du pokemon

    ranges = data.ranges
    #angle_increment = data.angle_increment
    angle_increment = radians(360)/len(ranges)

    first = True    
    for i in range(len(ranges)) :
        alpha = i*angle_increment

        if first:
            first = False
            x = -cos(alpha)*ranges[i]
            y = -sin(alpha)*ranges[i]
            obstacle_position = np.array([x, y], ndmin=2) 

            if i < 0: # Pls ignore
                obstacle_position = np.array([0, 0], ndmin=2) 

        else:
            x = -cos(alpha)*ranges[i]
            y = -sin(alpha)*ranges[i]
            obstacle_position_ = np.array([x, y], ndmin=2) 
            obstacle_position = np.append(obstacle_position, obstacle_position_, axis=0)

            if i < 0: # Pls ignore
                obstacle_position = np.array([0, 0], ndmin=2) 

    mutex.release()



# PD utilisé pour limiter la vitesse lorsqu'il y a un obstacle.
pid = PID(Kp, 0, Kd, setpoint)

rospy.init_node('drive_assist', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
rospy.Subscriber("/scan", LaserScan, lidar_callback)
cmd_vel_assisted_pub = rospy.Publisher("/cmd_vel_assisted", Twist, queue_size=10)


rate = rospy.Rate(100) # 100hz
while not rospy.is_shutdown():
    vehicule_length = 36.5/100
    length = setpoint # Longueur de la trajectoire
    offset = 32/100 # Offset avant/arriere
    largeur = 17/100 # Largeur vehicule
    pas = 1e-2 # resolution du polygon

    mutex.acquire()
    if speed > 0:
        if angle < radians(0.5) and angle > -radians(0.5) : # Dans ce cas, display un rectangle, car sinon div. par 0
            p_np = np.array([[-largeur, 0+offset],[largeur, 0+offset], [largeur, length+offset], [-largeur, length+offset]]) # p_np est le polygon numpy
            path = mpltPath.Path(p_np)

            dist_obstacle_old = inf_
            dist_obstacle_min = inf_ 

            for i in range(len(obstacle_position)):
                point_obstacle = np.array(obstacle_position[i], ndmin=2)

                if path.contains_points(point_obstacle) : # Vrai s'il y a un obstacle dans la trajectoire
                    dist_obstacle = point_obstacle[0,1] - offset #Distance en y.
                    
                    if dist_obstacle < dist_obstacle_old :
                        dist_obstacle_min = dist_obstacle
                    dist_obstacle_old = dist_obstacle
            

        else : 
            rayon = vehicule_length/tan(angle)
            angle_traj = length/rayon
            first = True

            if angle > 0: #Trajectoire  a gauche
                for theta in np.arange(0, angle_traj, pas): # Arc allant
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = (rayon+largeur) * sin(theta)+offset
                
                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)


                for theta in np.arange(angle_traj, 0, -pas): # Arc retour
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = (rayon-largeur) * sin(theta)+offset
                    
                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)

                
                path = mpltPath.Path(p_np)
                dist_obstacle_old = inf_
                dist_obstacle_min = inf_ 

                for i in range(len(obstacle_position)): #Parcourt tous les points et regarde si obstacle dans trajectoire.
                    point_obstacle = np.array(obstacle_position[i], ndmin=2)

                    if path.contains_points(point_obstacle) : # Vrai s'il y a un obstacle dans la trajectoire
                        dist_obstacle = distance_obstacle_arc(rayon, offset, point_obstacle, angle, debug=False) #Distance jusqu'a l'obstacle

                        if dist_obstacle < dist_obstacle_old :
                            dist_obstacle_min = dist_obstacle

                        dist_obstacle_old = dist_obstacle

                
            else : #Trajectoire  a droite
                for theta in np.arange(0, angle_traj, -pas): # Arc allant
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = (rayon+largeur) * sin(theta)+offset
                    
                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)


                for theta in np.arange(angle_traj, 0, pas): # Arc retour
                    x = (rayon-largeur) * cos(theta)-rayon 
                    y = (rayon-largeur) * sin(theta)+offset
                    
                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)

                
                path = mpltPath.Path(p_np)
                dist_obstacle_old = inf_
                dist_obstacle_min = inf_

                for i in range(len(obstacle_position)): #Parcourt tous les points et regarde si obstacle dans trajectoire.
                    point_obstacle = np.array(obstacle_position[i], ndmin=2)

                    if path.contains_points(point_obstacle) : # Vrai s'il y a un obstacle dans la trajectoire
                        dist_obstacle = distance_obstacle_arc(rayon, offset, point_obstacle, angle, debug=False) #Distance jusqu'a l'obstacle

                        if dist_obstacle < dist_obstacle_old :
                            dist_obstacle_min = dist_obstacle #Distance a l'obstacle le plus proche
                        dist_obstacle_old = dist_obstacle
                        
                    else:
                        rayon_obstacle = None

    elif speed < 0 :
        if angle < radians(0.5) and angle > -radians(0.5) : #Dans ce cas, display un rectangle.
            p_np = np.array([[-largeur, 0-offset],[largeur, 0-offset], [largeur, -length-offset], [-largeur, -length-offset]])

        else : 
            rayon = vehicule_length/tan(angle)
            angle_traj = length/rayon
            first = True

            if angle > 0:
                for theta in np.arange(0, angle_traj, pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = -((rayon+largeur) * sin(theta)+offset)

                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)


                for theta in np.arange(angle_traj, 0, -pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = -((rayon-largeur) * sin(theta)+offset)

                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)

            else :
                for theta in np.arange(0, angle_traj, -pas):
                    x = (rayon+largeur) * cos(theta)-rayon
                    y = -((rayon+largeur) * sin(theta)+offset)

                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)

                for theta in np.arange(angle_traj, 0, pas):
                    x = (rayon-largeur) * cos(theta)-rayon
                    y = -((rayon-largeur) * sin(theta)+offset)

                    if first :
                        p_np = np.array([x, y], ndmin=2)
                        first = False
                    else :
                        p_ = np.array([x, y], ndmin=2)
                        p_np = np.append(p_np, p_, axis=0)
    else:
        p_np = None

    mutex.release()

    if p_np is not None :
        
        if 0 :        
            plt.axis([-10,10,-10,10])
            plt.scatter(obstacle_position[:, 0], obstacle_position[:, 1], c ="blue") # Plot de l'obstacle
            #plt.scatter(point[:, 0], point[:, 1], c ="blue") # Plot de l'obstacle
            #plt.scatter(-rayon, offset, c ="blue") # Plot du centre du cercle
            plt.scatter(p_np[:,0], p_np[:,1], c ="green")
            #plt.plot([-rayon, point[0,0]], [offset, point[0,1]])
            plt.ion()
            plt.show()
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
            
        #print(dist_obstacle_min)

        # PID
        vel_assisted = Twist()
        control = pid(dist_obstacle_min)
        
        


        if control > 0 :
            max_speed_allowed = abs(max_speed-control)

            if control > max_speed :
                max_speed_allowed = 0


        #print(max_speed_allowed)

        if dist_obstacle_min == inf_: # Si pas d'obstacle go max speed.
            max_speed_allowed = max_speed

        if speed > max_speed_allowed : # Limite la vitesse publiée par cmd_vel
            vel_assisted.linear.x = max_speed_allowed

        else :
            vel_assisted.linear.x = speed


        

        vel_assisted.angular.z = -degrees(angle)
        cmd_vel_assisted_pub.publish(vel_assisted)
            

    rate.sleep()

