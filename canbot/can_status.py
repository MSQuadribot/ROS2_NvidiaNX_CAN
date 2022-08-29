
from re import sub
import rclpy

from interfaces.msg import BusCan

from rclpy.node import Node

import subprocess

class GetStatus(Node):

    def __init__(self):
        '''
        Create a Node that will retreives CAN bus data from the can_data topic.
        The data are then used to compute car's status in a readable format.
        Those information are then printed out and are easily accessible for the user.
        '''
        
        super().__init__('get_status')
        self.subscription = self.create_subscription(BusCan, 'can_data', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

        self.angle = None
        self.speed = None
        self.brake = None
        self.battery = None
        self.error = None
        
    def listener_callback(self, msg):
        '''
        Retreive specific messages from the CAN bus in order to get precise data.
        Then update the class' variables at all time.
        '''

        subprocess.call("clear")

        if msg.arbitration_id == 300:
            self.brake = msg.data[2]

        if msg.arbitration_id == 301:
            self.speed = msg.data[2]


        if msg.arbitration_id == 302:
            angle_1 = msg.data[2]
            angle_2 = msg.data[3]

            if angle_2 == 1:
                self.angle = angle_1 + 255
            
            elif angle_2 == 0:
                self.angle = angle_1
            
            elif angle_2 == 255:
                self.angle = angle_1 - 254

            elif angle_2 == 254:
                self.angle = angle_1 -509

        if msg.arbitration_id == 303:
            bat_1 = str(hex(msg.data[0]))[2:4]
            bat_2 = str(hex(msg.data[1]))[2:4]

            self.battery = int('0x' + bat_2 + bat_1,16)
        
        if msg.arbitration_id == 304:
            for i in msg.data:
                if i != 0:
                    self.error = i
                    break

        if self.speed != None:
            self.get_logger().info('Speed: ' + str(self.speed//10) + 'km/h')
        if self.brake != None:
            self.get_logger().info('Brake :' + str(self.brake))
        if self.angle != None:
            self.get_logger().info('Angle: ' + str(self.angle//10) + '°')
        if self.battery != None:
            self.get_logger().info('Battery: ' + str(self.battery//10) + '%')
        self.get_logger().info('Error: ' + str(self.error))
        
def main():

    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    rclpy.init()

    get_status = GetStatus()

    try:
        rclpy.spin(get_status)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_status.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
