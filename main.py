import os
import sys

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Vector3

import time

import json

debug = True


loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

# sensor variable initialization
imu_global = {}
color_global = {}
accelerometer_global = {}
ambient_global = {}
encoder_global = {}

received = 0x00     # received byte - fully received at 0x1f

# send stuff variable initialization


# sensor publisher
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sphero_node')
        self.publisher_ = self.create_publisher(
            String,
            'chatter',  # publish to chatter channel
            10)

        self.subscription = self.create_subscription(
            Int8MultiArray,
            'drive',   # listen on drive channel
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def send(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        # if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, msg):
        if debug: print("heard")
        # motor code here

        drive_data = msg.data

        # separate left motor and right motor
        right_drive = drive_data[1]
        left_drive = drive_data[0]


        print(str(left_drive) + str(right_drive))

        print(str(drive_data))

        #asyncio.get_event_loop().run_until_complete(
        #    rvr.drive_tank_normalized(
        #        left_velocity=left_drive,
        #        right_velocity=right_drive
        #    )
        #)

rclpy.init(args=None)
publisher = MinimalPublisher()

# Check if all values are collected
def checkData():
    global received
    if received == 31:
        received = 0
        publisher.send(
            json.dumps({
                    **imu_global,
                    **color_global,
                    **accelerometer_global,
                    **ambient_global,
                    **encoder_global
            })
        )
        if debug: print("spinning")
        rclpy.spin_once(publisher, timeout_sec=0.01)
        if debug: print('publish')



async def imu_handler(imu_data):
    #print('IMU data response: ', imu_data)
    if debug: print('IMU data received')
    global imu_global
    global received
    imu_global = imu_data
    received = received | (1)
    #print(received)
    checkData()


async def color_detected_handler(color_detected_data):
    #print('Color detection data response: ', color_detected_data)
    if debug: print('Color detection data received')
    global color_global
    global received
    color_global = color_detected_data
    received = received | (1 << 1)
    #print(received)
    checkData()


async def accelerometer_handler(accelerometer_data):
    #print('Accelerometer data response: ', accelerometer_data)
    if debug: print('Accelerometer data received')
    global accelerometer_global
    global received
    accelerometer_global = accelerometer_data
    received = received | (1 << 2)
    #print(received)
    checkData()


async def ambient_light_handler(ambient_light_data):
    #print('Ambient light data response: ', ambient_light_data)
    if debug: print('Ambient light data received')
    global ambient_global
    global received
    ambient_global = ambient_light_data
    received = received | (1 << 3)
    #print(received)
    checkData()

async def encoder_handler(encoder_data):
    #print('Encoder data response: ', encoder_data)
    if debug: print('Encoder data received')
    global encoder_global
    global received
    encoder_global = encoder_data
    received = received | (1 << 4)
    #print(received)
    checkData()

async def spin_ros():
    while True:
        if debug: print("~~~~~LISTENING~~~~~")
        await rclpy.spin_once(publisher)
        if debug: print("~~~~~HEARD~~~~~")


async def main():
    """ This program demonstrates how to enable multiple sensors to stream.
    """

    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    if debug: print("Starting imu handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler
    )
    if debug: print("Starting color handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_detected_handler
    )
    if debug: print("Starting accelerometer handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.accelerometer,
        handler=accelerometer_handler
    )
    if debug: print("Starting ambient light handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=ambient_light_handler
    )
    if debug: print("Starting encoder handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.encoders,
        handler=encoder_handler
    )
    if debug: print("Starting sensor control")

    #await rvr.sensor_control.start(interval=250)
    await rvr.sensor_control.start(interval=1000)

    if debug: print("Ros listener spinning up")
    #await spin_ros()
    if debug: print("spin complete")



    # The asyncio loop will run forever to allow infinite streaming.


if __name__ == '__main__':
    try:

        asyncio.ensure_future(
            main()
        )
        loop.run_forever()

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )

    finally:
        if loop.is_running():
            loop.close()