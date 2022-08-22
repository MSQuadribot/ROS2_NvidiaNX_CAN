import keyboard
import time
import rclpy

from rclpy.node import Node
from interfaces.msg import CarControl

def keyboard_listener(mode):
    '''
    This function will listen the keyboard input and update the variables.
    Thus the user can control the variable of the car.
    '''

    key = keyboard.read_key()
    if key in ["1","2","3","4","5","6","7","8"]:
        mode = int(key)

    return(mode)

class CanInput(Node):

    def __init__(self):
        '''
        This will define the Node with both a publisher.
        The goal is to send the data input from the keyboard to the controller Node.
        It has to send the data as fast as possible
        '''

        super().__init__('can_input')
        self.publisher = self.create_publisher(CarControl, 'keyboard_input', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # The data that will be sent to the car are defined inside the class for the publisher
        self.mode = 1       # 8 is external mode
        self.direction = 0  # 0 = forward, 1 = backward, 2 = neutral
        self.speed = 0      # Speed set to 0 km/h
        self.steering = 0   # No steering
        self.brake = 4      # No brake
    
    def timer_callback(self):
        '''
        This callback take the keyboard input into account.
        The data are then updated and send using a specific message type.
        '''

        self.mode= keyboard_listener(self.mode)
        msg = CarControl()
        msg.mode = self.mode
        msg.direction = self.direction
        msg.speed = self.speed
        msg.steering = self.steering
        msg.brake = self.brake
        self.publisher.publish(msg)
        self.get_logger().info("Publishing :" + str(msg.mode) + " " + str(msg.direction) + " " + str(msg.speed) + " " + str(msg.steering) + " " + str(msg.brake))
        time.sleep(0.02)


def main():
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    rclpy.init()

    can_input = CanInput()

    try:
        rclpy.spin(can_input)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()