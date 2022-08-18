import can

import rclpy
from rclpy.node import Node
from interfaces.msg import CarControl
from std_msgs.msg import String

class CanController(Node):
    '''
    This node is responsible for sending the CAN messages to the car.
    It has both a publisher and a subscriber. The subscriber is responsible for receiving the keyboard input from the user.
    The publisher is responsible for sending the CAN messages to the car.
    '''

    def __init__(self, bus):
        '''
        This will define the Node with both a publisher and a subscriber.
        The publisher does not send any ROS messages.
        The subscriber receives the keyboard input from the user.
        '''

        super().__init__('can_controller')
        self.subscription = self.create_subscription(CarControl, 'keyboard_input', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        self.publisher = self.create_publisher(String, 'Can_msg',10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = bus

        # The data that will be sent to the car are defined inside the class for the publisher
        self.mode = 8       # 8 is external mode
        self.direction = 2  # 0 = forward, 1 = backward, 2 = neutral
        self.speed = 0      # Speed set to 0 km/h
        self.steering = 0   # No steering
        self.brake = 0      # No brake
        self.alive = 0

    def listener_callback(self, msg):
        '''
        The subscriber received the input from the user.
        The data are then updated and are ready to be used by the publisher.
        '''

        self.get_logger().info("Receiving :" + str(msg.mode) + " " + str(msg.direction) + " " + str(msg.speed) + " " + str(msg.steering) + " " + str(msg.brake))
        self.mode = msg.mode
        self.direction = msg.direction
        self.speed = msg.speed
        self.steering = msg.steering * 10
        self.brake = msg.brake
    
    def timer_callback(self):
        '''
        The publisher sends the CAN messages to the car.
        Some conversion are however needed to send the correct data to the car.
        Information are sent every 20 ms even if data have not changed.
        '''

        if self.brake > 0:
            message = can.Message(arbitration_id=0x200, is_extended_id=False,data=[8, 0, 0, 0, 0, 0, self.brake, self.alive])
            self.bus.send(message, timeout=0)
        
        else:
            if self.steering>0:
                if self.steering < 255:
                    bytangle0 = self.steering
                    bytangle1 = 0
                elif self.steering>255:
                    bytangle0 = self.steering-255
                    bytangle1 = 1
            else:
                if self.steering > -255:
                    bytangle0 = 255-abs(self.steering)-1
                    bytangle1 = 255
                elif self.steering < -255:
                    bytangle0 = 255-abs(self.steering+255)-1
                    bytangle1 = 254
            
            message = can.Message(arbitration_id=0x200, is_extended_id=False,data=[self.mode, self.direction, self.speed, 0, bytangle0, bytangle1, 0x00, self.alive])
            self.bus.send(message, timeout=0)

        if self.alive < 255:
            self.alive += 1
        else:
            self.alive = 0



def main():
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    rclpy.init()

    bus = can.ThreadSafeBus(interface='socketcan', channel="vcan0",timeout=0.01)

    can_controller = CanController(bus)

    try:
        rclpy.spin(can_controller)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()