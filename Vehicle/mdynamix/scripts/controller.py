#!/usr/bin/env python3

import RPi.GPIO as GPIO
import numpy as np
import rospy
from geometry_msgs.msg import Twist


class parameters():
    def __init__(self):
        #PWM parameters
        self.servo_pin = 18
        self.motor_pin = 13
        self.frequency = 200 # Fréquence entre 50 et 200ms pour les servos/esc, 200 = meilleure précision

        self.speed = 0
        self.brake = 0
        self.steeringWheel = 0


def cmd_vel_callback(data):
    #data.linear.x représente la vitesse linéaire du véhicle donnée en %. 100% = max thrust, 0% = arrêt, -100% = full reverse.
    max_speed = 100
    if data.linear.x > max_speed:
        params.speed = max_speed

    elif data.linear.x < -1*max_speed:
        params.speed = -1*max_sèeed

    else:
        params.speed = data.linear.x # Vitesse vehicule


    #data.angular.z représente l'angle des roues avant. 0° = tout droit, -30° gauche et 30° droite. La valeur de 30° est limitée par les butées du véhicule.
    max_angle = 30
    if data.angular.z > max_angle:
        params.steeringWheel = max_angle

    elif data.angular.z < -1*max_angle:
        params.steeringWheel = -1*max_angle

    else:
        params.steeringWheel = data.angular.z # Angle volant


def main():
    node = 'mdynamix'
    rospy.init_node(node, anonymous=True)
    rospy.loginfo('Node %s initialized.', node)
    rospy.Subscriber("cmd_vel_assisted", Twist, cmd_vel_callback)

    # Pin Setup: Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(params.servo_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(params.motor_pin, GPIO.OUT, initial=GPIO.HIGH)
    # Initie le pwm pour le servo
    servo_steeringWheel = GPIO.PWM(params.servo_pin, params.frequency)
    esc_motor = GPIO.PWM(params.motor_pin, params.frequency)
    # Start le servo au centre, 10 = min, 15 = center, 20 = max (voir duty cycle servo sur google)
    servo_steeringWheel.start(15*params.frequency/100)
    rospy.loginfo('Steering servo centered.')

    # Start l'esc au minimum, 10 = reverse, 15 = center, 20 = full
    esc_motor.start(15*params.frequency/100)
    rospy.loginfo('ESC started.')

    
    try :
        while not rospy.is_shutdown():
            #Conversion de l'angle -30/30 en duty cycle correspondant
            offset_gyro=0.8 #Offset pour que l'angle des roues valent bien 0 quand l'angle est de 0. (Il faudrait modifier physiquement sur le servo)
            dutyServo = np.interp(params.steeringWheel,[-40, 40],[(10-offset_gyro)*params.frequency/100, (20)*params.frequency/100])
            servo_steeringWheel.ChangeDutyCycle(dutyServo)


            #Conversion de la vitesse -100/100
            dutyESC = np.interp(params.speed,[-100, 100],[10*params.frequency/100, 20*params.frequency/100])
            esc_motor.ChangeDutyCycle(dutyESC)
            #print(dutyESC/(params.frequency/100))


    finally:
        servo_steeringWheel.stop()
        GPIO.cleanup()
        rospy.loginfo('Shutting down node %s.', node)



if __name__ == '__main__':
    params = parameters()
    main()

