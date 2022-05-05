#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int64, Float64

# Import library for Pololu driver
from dual_g2_hpmd_rpi import Motor, MAX_SPEED

# Import Encoder Librart
import pigpio


# Set Pololu Driver Pin
_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1PWM = 12
_pin_M2PWM = 13
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25

# Set Encoder Pin
E1_Pin1 = 14
E1_Pin2 = 15
E2_Pin1 = 4
E2_Pin2 = 16

# Set Encoder Refresh Rate
TICKS_PER_ROTATION = 187  #11/motor rotation and 1:17 reduction

###########################################################################
###                     Encoder Class Define                            ###
###########################################################################

class decoder:

    def __init__(self, pi, gpioA, gpioB, callback):

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):

      
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level;

        if gpio != self.lastGpio: # debounce
            self.lastGpio = gpio
            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.callback(1)
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.callback(-1)

    def cancel(self):
        self.cbA.cancel()
        self.cbB.cancel()

###########################################################################
###                     Encoder Class Define                            ###
###########################################################################

class dcbot_motor_driver: 

    def __init__(self):
        rospy.init_node('dcbot_motor_driver', anonymous=True)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        self.motor_port = rospy.get_param('~motor_port')
        self.enc_rate = rospy.get_param('~enc_rate')

        rospy.loginfo("Set motor port to %s" % self.motor_port)
        rospy.loginfo("Set encoder rate to %s Hz" % self.enc_rate)

        if self.motor_port == 1:
            pin_FLT = _pin_M1FLT
            pin_PWM = _pin_M1PWM
            pin_EN = _pin_M1EN
            pin_DIR = _pin_M1DIR
            Epin1 = E1_Pin1
            Epin2 = E1_Pin2
            self.REV = 1
        else: 
            pin_FLT = _pin_M2FLT
            pin_PWM = _pin_M2PWM
            pin_EN = _pin_M2EN
            pin_DIR = _pin_M2DIR
            Epin1 = E2_Pin1
            Epin2 = E2_Pin2
            self.REV = -1
        self.pos = 0
        self.spd = 0
        self.motor = Motor(pin_PWM, pin_DIR, pin_EN, pin_FLT)
        self.enc_pub = rospy.Publisher('enc', Int64, queue_size=10)
        self.spd_pub = rospy.Publisher('state', Float64, queue_size=10)
        rospy.Subscriber("control_effort", Float64, self.motor_control)
        pi = pigpio.pi()
        decoder(pi, Epin1, Epin2, self.enc_cbk)
        self.spd_publisher()
        rospy.spin()

    def motor_control(self, msg):

        self.spd += msg.data
        # Limit max speed to 460 (ABS MAX = 480)
        if self.spd < 5 and self.spd > -5:
            self.spd = 0
        elif self.spd > 460:
            self.spd = 460
        elif self.spd < -460:
            self.spd = -460

        # Apply Speed
        try:
            self.motor.setSpeed(self.spd*self.REV)

        except:
            rospy.loginfo("Driver %s fault!" % self.motor.getFault())
   
    def enc_cbk(self,way):
        self.pos += way
        self.enc_pub.publish(self.pos*self.REV)

    def spd_publisher(self):
        loop_length = (1.00 / float(self.enc_rate)) / 60
        rate = rospy.Rate(self.enc_rate)  #50Hz,0.02s per loop
        last_pos = self.pos
        while not rospy.is_shutdown():
            #rotate ticks / loop per minute / ticks per rotation == RPM 
            speed = float(self.pos - last_pos) / loop_length / TICKS_PER_ROTATION
            last_pos = self.pos
            self.spd_pub.publish(speed*self.REV)
            rate.sleep()



if __name__ == '__main__':
    try:
        dcbot_motor_driver()
    except rospy.ROSInterruptException:
        pass
