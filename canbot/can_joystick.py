import pygame
import time
import rclpy

from rclpy.node import Node
from interfaces.msg import CarControl

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joystick_input(joystick, direction,speed, steering, brake):

    brake = 0

    pygame.event.pump()
    y = joystick.get_axis(0)
    x = joystick.get_axis(1)

    if joystick.get_button(10):
        brake = 1
    if joystick.get_button(11):
        brake = 2
    
    if y < 0.05 and y > -0.05:
        y = 0
    
    if x < 0.05 and x > -0.05:
        x = 0
    
    speed = remap(-x,-1,1,-191,190)
    steering = remap(y,-1,1,-37,38)

    if speed < 0:
        direction = 1
        speed = -speed
    else :
        direction = 0

    return(direction,int(speed),int(steering),brake)


class CanInput(Node):

    def __init__(self, mode, joystick):
        '''
        This will define the Node with both a publisher.
        The goal is to send the data input from the keyboard to the controller Node.
        It has to send the data as fast as possible
        '''

        super().__init__('can_input')
        self.publisher = self.create_publisher(CarControl, 'keyboard_input', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joystick = joystick

        # The data that will be sent to the car are defined inside the class for the publisher
        self.mode = mode       # 8 is external mode
        self.direction = 2  # 0 = forward, 1 = backward, 2 = neutral
        self.speed = 0      # Speed set to 0 km/h
        self.steering = 0   # No steering
        self.brake = 0      # No brake
    
    def timer_callback(self):
        '''
        This callback take the keyboard input into account.
        The data are then updated and send using a specific message type.
        '''

        self.direction,self.speed,self.steering,self.brake = joystick_input(self.joystick, self.direction,self.speed,self.steering,self.brake)
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

    mode = int(input("What is the required mode for the car?"))

    pygame.init()

    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joysticks found.")

    joystick = pygame.joystick.Joystick(0)

    joystick.init()

    can_input = CanInput(mode, joystick)

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