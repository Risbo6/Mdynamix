#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time


last_callback = 0
current_perf_counter = 0
watchdog_value = 100 # Le watchdog proc lorsque rien n'a été publié sur cmd_vel pendant 100 ms.

def cmd_vel_callback(data):
    global last_callback
    last_callback = time.perf_counter()



rospy.init_node('watchdog', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)


rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    twist = Twist()
    current_perf_counter = time.perf_counter()

    if abs(current_perf_counter-last_callback) > watchdog_value/1000 :
        cmd_vel_pub.publish(twist)


    rate.sleep()
