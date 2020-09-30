import rclpy
from rclpy.node import Node

from std_msgs.msg import Byte


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Byte, 'drive', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_byte(self, data):
        msg = Byte()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    while True:
        inp = input("byte to send: ")
        try:
            inp=int(inp, 16)
            print(inp)
            byte = inp.to_bytes(1, byteorder='big')
            rightbyte = (byte[0] & b'\x0f'[0]).to_bytes(1, byteorder='big')
            leftbyte = ((byte[0] & b'\xf0'[0]) >> 4).to_bytes(1, byteorder='big')
            print(leftbyte)
            print(rightbyte)
            print()

        except ValueError:
            print("invalid hex")
            continue
        minimal_publisher.send_byte(byte)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()