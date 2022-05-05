#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

# Import library for Pololu driver
import time
from dual_g2_hpmd_rpi import motors, MAX_SPEED

LEFT_DIRECTION = 2
RIGHT_DIRECTION = -2

class dcbot_base_driver: 

    def __init__(self):
        rospy.init_node('dcbot_base_driver', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        rospy.spin()

    def callback(self, msg):

        linear_speed = float(msg.linear.x)
        angular_speed = float(msg.angular.z)
        # Limit max speed to 100
        if linear_speed < 5 and linear_speed > -5:
            linear_speed = 0
        elif linear_speed > MAX_SPEED: 
            linear_speed = MAX_SPEED
        elif linear_speed < -MAX_SPEED:
            linear_speed = -MAX_SPEED
        if angular_speed < 5 and angular_speed > -5:
            angular_speed = 0
        elif angular_speed > MAX_SPEED:
            angular_speed = MAX_SPEED
        elif angular_speed < -MAX_SPEED:
            angular_speed = -MAX_SPEED
        # Speed Formula
        speed_left = (linear_speed - angular_speed) * LEFT_DIRECTION 
        speed_right = (linear_speed + angular_speed) * RIGHT_DIRECTION 
        rospy.loginfo([speed_left, speed_right])
        # Apply Speed
        try:
            motors.motor1.setSpeed(speed_left)
            motors.motor2.setSpeed(speed_right)
        except DriverFault as e:
            print("Driver %s fault!" % e.driver_num)
    
    # Driver Fault Class
    class DriverFault(Exception):
        def __init__(self, driver_num):
            self.driver_num = driver_num

    def raiseIfFault():
        if motors.motor1.getFault():
            raise DriverFault(1)
        if motors.motor2.getFault():
            raise DriverFault(2)



if __name__ == '__main__':
    try:
        dcbot_base_driver()
    except rospy.ROSInterruptException:
        pass
