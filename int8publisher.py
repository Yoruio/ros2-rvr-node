import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'drive', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_ints(self, data):
        msg = Int8MultiArray()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    while True:
        arr = [None] * 2
        arr[0] = int(input("left motor: "))
        arr[1] = int(input("right motor: "))
        print (arr)
        minimal_publisher.send_ints(arr)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()