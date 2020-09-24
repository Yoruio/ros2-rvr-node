import os
import sys

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json


loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

imu_global = {}
color_global = {}
accelerometer_global = {}
ambient_global = {}
encoder_global = {}

received = 0x00

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

    def send(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

rclpy.init(args=None)
publisher = MinimalPublisher()

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
        print('publish')



async def imu_handler(imu_data):
    #print('IMU data response: ', imu_data)
    print('IMU data received')
    global imu_global
    global received
    imu_global = imu_data
    received = received | (1)
    print(received)
    checkData()


async def color_detected_handler(color_detected_data):
    #print('Color detection data response: ', color_detected_data)
    print('Color detection data received')
    global color_global
    global received
    color_global = color_detected_data
    received = received | (1 << 1)
    print(received)
    checkData()


async def accelerometer_handler(accelerometer_data):
    #print('Accelerometer data response: ', accelerometer_data)
    print('Accelerometer data received')
    global accelerometer_global
    global received
    accelerometer_global = accelerometer_data
    received = received | (1 << 2)
    print(received)
    checkData()


async def ambient_light_handler(ambient_light_data):
    #print('Ambient light data response: ', ambient_light_data)
    print('Ambient light data received')
    global ambient_global
    global received
    ambient_global = ambient_light_data
    received = received | (1 << 3)
    print(received)
    checkData()

async def encoder_handler(encoder_data):
    #print('Encoder data response: ', encoder_data)
    print('Encoder data received')
    global encoder_global
    global received
    encoder_global = encoder_data
    received = received | (1 << 4)
    print(received)
    checkData()


async def main():
    """ This program demonstrates how to enable multiple sensors to stream.
    """

    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_detected_handler
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.accelerometer,
        handler=accelerometer_handler
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=ambient_light_handler
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.encoders,
        handler=encoder_handler
    )

    #await rvr.sensor_control.start(interval=250)
    await rvr.sensor_control.start(interval=1000)

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