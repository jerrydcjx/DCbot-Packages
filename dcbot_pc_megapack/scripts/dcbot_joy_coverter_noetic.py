#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
class dcbot_joy_node: 

    def __init__(self):
        rospy.init_node('dcbot_joy_cmd', anonymous=True)
        self.cmd = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("joy", Joy, self.callback)
        rospy.spin()

    def callback(self, msg):

        self.cmd.linear.x = float(msg.axes[3]*0.25*(msg.buttons[5]+msg.buttons[7]+1))
        self.cmd.linear.y = float(0)
        self.cmd.linear.z = float(0)
        self.cmd.angular.x = float(0)
        self.cmd.angular.y = float(0)
        self.cmd.angular.z = -float(msg.axes[0]*2*(msg.buttons[5]+msg.buttons[7]+1))
        rospy.loginfo(self.cmd)
        self.pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        dcbot_joy_node()
    except rospy.ROSInterruptException:
        pass