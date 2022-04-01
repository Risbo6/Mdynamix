#!/usr/bin/python3

import pygame
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('logitech_g29', anonymous=True)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

pygame.init()

# Initialize the joysticks.
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
rospy.loginfo(f'{joystick.get_name()} initialized.')

rate = rospy.Rate(100) 
while not rospy.is_shutdown():
    # ROS message
    twist = Twist()

    #Get events
    for event in pygame.event.get(): # User did something.
        pass

    buttons = joystick.get_numbuttons()

    for i in range(buttons):
        button = joystick.get_button(i)
        if i == 14:
            if button :
                forward = 1
            else:
                forward = 0

        elif i == 15:
            if button :
                reverse = 1
            else :
                reverse = 0

    #print(forward, reverse)


    try:
        jid = joystick.get_instance_id()
    except AttributeError:
        jid = joystick.get_id()


    for i in range(joystick.get_numaxes()):
        axis = joystick.get_axis(i)

        if i == 0:
            twist.angular.z = axis*30

            if twist.angular.z > 30 :
                twist.angular.z = 30

            if twist.angular.z < -30 :
                twist.angular.z = -30


        elif i == 2:
            if forward :
                twist.linear.x = (axis-1)*-10
            elif reverse :
                twist.linear.x = (axis-1)*30
            else :
                twist.linear.x = 0


    cmd_vel_pub.publish(twist)

    rate.sleep()



