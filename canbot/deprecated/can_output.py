import rclpy
from rclpy.node import Node
from interfaces.msg import CarControl

class KeyListener(Node):

    def __init__(self):
        super().__init__('key_listener')
        self.subscription = self.create_subscription(CarControl, 'keyboard_input', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Publishing :" + str(msg.mode) + " " + str(msg.direction) + " " + str(msg.speed) + " " + str(msg.steering) + " " + str(msg.brake))

def main():

    rclpy.init()

    key_listener = KeyListener()

    try:
        rclpy.spin(key_listener)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()