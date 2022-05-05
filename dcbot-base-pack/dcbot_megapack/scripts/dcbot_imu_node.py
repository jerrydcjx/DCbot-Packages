#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Imu

#import SMBus module of I2C
import smbus                    

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def dcbot_imu_publisher():
    rospy.init_node('dcbot_imu', anonymous=True)
    MPU_Init()
    pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    Rate = rospy.Rate(10) # 10hz
    imu = Imu()

    while not rospy.is_shutdown():
            
        Ax = read_raw_data(ACCEL_XOUT_H)/16384.0
        Ay = read_raw_data(ACCEL_YOUT_H)/16384.0
        Az = read_raw_data(ACCEL_ZOUT_H)/16384.0

        Gx = read_raw_data(GYRO_XOUT_H)/131.0
        Gy = read_raw_data(GYRO_YOUT_H)/131.0
        Gz = read_raw_data(GYRO_ZOUT_H)/131.0

        imu.linear_acceleration.x = Ax
        imu.linear_acceleration.y = Ay
        imu.linear_acceleration.z = Az

        imu.angular_velocity.x = Gx
        imu.angular_velocity.y = Gy
        imu.angular_velocity.z = Gz

        imu.header.stamp = rospy.Time.now()

        rospy.loginfo(imu)
        pub.publish(imu)
        Rate.sleep()


if __name__ == '__main__':
    try:
        dcbot_imu_publisher()
    except rospy.ROSInterruptException:
        pass
