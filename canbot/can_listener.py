import rclpy
from rclpy.node import Node
from interfaces.msg import BusCan

class CanListener(Node):

    def __init__(self):
        super().__init__('can_listener')
        self.subscription = self.create_subscription(BusCan, 'can_data', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data == []:
            self.get_logger().info('Subscribing: empty message')
        else:
            self.get_logger().info('Subscribing: ' + str(msg.arbitration_id) + ', '.join(str(int(hex(b),16)) for b in msg.data))

def main():

    rclpy.init()

    can_listener = CanListener()

    try:
        rclpy.spin(can_listener)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()