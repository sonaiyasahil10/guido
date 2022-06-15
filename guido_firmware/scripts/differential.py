#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from math import pi

leftEn = 25        #   
rightEn = 17       #   

leftBackward =  24    #   
leftForward =   23    #   
rightForward =  27    #   
rightBackward = 22    #   

motor_rpm = 60             #   max rpm of motor on full voltage 
wheel_diameter = 0.1       #   in meters
wheel_separation = 0.36    #   in meters
max_pwm_val = 100          #   100 for Rpi and 255 for Arduino
min_pwm_val = 30           #   min pwm needed for robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBackward, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBackward, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)

def stop():
    #print('stopping')
    pwmL.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)

def forward(left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    #print('going forward')
    lspeedPWM = max(min(((left_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((right_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.LOW)

def backward(left_speed, right_speed):
    #print('going backward')
    lspeedPWM = max(min(((left_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((right_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)

def left(left_speed, right_speed):
    #print('turning left')
    lspeedPWM = max(min(((left_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((right_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.LOW)

def right(left_speed, right_speed):
    #print('turning right')
    lspeedPWM = max(min(((left_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((right_speed/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.HIGH)
    
def callback(data):
    
    global wheel_radius
    global wheel_separation
    
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    #print(str(linear)+"\t"+str(angular))
    
    VrplusV1 = 2*linear_vel
    VrminusV1 = angular_vel * wheel_separation
    
    right_vel = ( VrplusV1 + VrminusV1) / 2   # Right wheel velocity along the ground
    left_vel = VrplusV1 - right_vel           # left wheel velocity along the ground 

    '''rplusl  = ( 2 * linear_vel ) / wheel_radius
    rminusl = ( angular_vel * wheel_separation ) / wheel_radius
    
    right_omega = ( rplusl + rminusl ) / 2
    left_omega  = rplusl - right_omega 
    
    right_vel = right_omega * wheel_radius
    left_vel  = left_omega  * wheel_radius'''
    
    #print (str(left_vel)+"\t"+str(right_vel))
    '''
    left_speed  = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    right_speed = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    '''
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
    elif (left_vel >= 0.0 and right_vel >= 0.0):
        forward(abs(left_vel), abs(right_vel))
    elif (left_vel <= 0.0 and right_vel <= 0.0):
        backward(abs(left_vel), abs(right_vel))
    elif (left_vel < 0.0 and right_vel > 0.0):
        left(abs(left_vel), abs(right_vel))
    elif (left_vel > 0.0 and right_vel < 0.0):
        right(abs(left_vel), abs(right_vel))
    else:
        stop()
        
def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('Guido Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    listener()
