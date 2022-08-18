import rclpy
from rclpy.node import Node
from interfaces.msg import BusCan

class CanListener(Node):
    '''
    This node serves only one simple purpose.
    It will listen to the canbus and print out the data.
    The Node allows the user to easely read the data.
    '''

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
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

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