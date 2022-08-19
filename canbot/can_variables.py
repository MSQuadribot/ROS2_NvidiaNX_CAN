from tkinter.messagebox import NO
import rclpy

from interfaces.msg import BusCan

from rclpy.node import Node

class GetAngle(Node):

    def __init__(self):
            super().__init__('get_angle')
            self.subscription = self.create_subscription(BusCan, 'can_data', self.listener_callback, 10)
            self.subscription # prevent unused variable warning

            self.angle = None
            self.speed = None
            self.battery = None
            self.error = None
        
    def listener_callback(self, msg):

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
        
        if msg.arb == 304:
            for i in msg.data:
                if i != 0:
                    self.error = i
                    break
        
        self.get_logger().info('Speed' + str(self.speed//10) + 'km/h')
        self.get_logger().info('Angle: ' + str(self.angle//10) + '°')
        self.get_logger().info('Battery: ' + str(self.battery//10) + '%')
        self.get_logger().info('Error: ' + str(self.error))
        
def main():

    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    rclpy.init()

    get_angle = GetAngle()

    try:
        rclpy.spin(get_angle)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_angle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
