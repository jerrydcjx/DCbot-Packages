#!/usr/bin/env python

import rospy
from math import sin, cos, pi
from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class dcbot_base_controller: 

    def __init__(self):
        rospy.init_node('dcbot_base_controller', anonymous=True)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        #Get Parameters
        self.controller_rate = rospy.get_param('controller_rate', 10) #10Hz
        self.tpr = rospy.get_param('tpr', 187)
        self.base_width = rospy.get_param('base_width', 0.17)
        self.wheel_d = rospy.get_param('wheel_diameter', 0.032)

        self.base_frame_id = rospy.get_param('base_frame_id','base_link')
        self.odom_frame_id = rospy.get_param('odom_frame_id','odom')

        self.loop_time = 1.0 / float(self.controller_rate)

        #Init Internal
        self.left_enc = 0
        self.left_last = 0
        self.right_enc = 0
        self.right_last = 0
        self.th = 0
        self.x = 0
        self.y = 0
        self.dx = 0
        self.dr = 0
        self.tpm = self.tpr/(self.wheel_d * pi) 
        self.then = rospy.Time.now()
        
        self.t_delta = rospy.Duration(1.0/self.controller_rate)
        self.t_next = rospy.Time.now() + self.t_delta

        rospy.Subscriber("left_enc", Int64, self.LeftCallback)
        rospy.Subscriber("right_enc", Int64, self.RightCallback)
        rospy.Subscriber("cmd_vel", Twist, self.Spd_Callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        self.left_pub = rospy.Publisher("left_spd", Float64, queue_size=10)
        self.right_pub = rospy.Publisher("right_spd", Float64, queue_size=10)


    #############################################################################
    def spin(self):
    #############################################################################
        rate = rospy.Rate(self.controller_rate) 
        rospy.logdebug("Odom Update Running")
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            
            # calculate odometry
            d_left = (self.left_enc - self.left_last) / self.tpm
            d_right = (self.right_enc - self.right_last) / self.tpm
            self.right_last = self.right_enc
            self.left_last = self.left_enc

            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / self.loop_time
            self.dr = th / self.loop_time
           
             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th

            rospy.loginfo('Enc Left: %s , Right: %s' % (d_left,d_right))

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)



    def LeftCallback(self, msg):
        self.left_enc = msg.data
        #rospy.loginfo('recieve Left enc: %s',self.left_enc)

    def RightCallback(self, msg):
        self.right_enc = msg.data

    def Spd_Callback(self, msg):
        rad_spd = msg.angular.z                     #Rad/s
        rotation = rad_spd * self.base_width / 2    #M/s
        linear = msg.linear.x
        left_spd_rpm = (linear + rotation) / (self.wheel_d * pi) * 60
        right_spd_rpm = (linear - rotation) / (self.wheel_d * pi) * 60
        rospy.logdebug("output left_spd: %s  right_spd: %s " % (left_spd_rpm,right_spd_rpm))
        self.left_pub.publish(left_spd_rpm)
        self.right_pub.publish(right_spd_rpm)

if __name__ == '__main__':
    try:
        dc_base = dcbot_base_controller()
        dc_base.spin()
    except rospy.ROSInterruptException:
        pass
