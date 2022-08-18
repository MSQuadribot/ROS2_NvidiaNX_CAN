'''This module is used to read the data from the can bus and display them in a readable way.'''

import can
import array
import rclpy

from rclpy.node import Node
from interfaces.msg import BusCan

idlist = {0x300:300,
    0x301:301,
    0x302:302,
    0x303:303,
    0x304:304,
    0x305:305,
    0x200:200}

def can_reader(canbus):
    '''
    This function listen to the bus can from the car and print out the required data.
    It has been specialy written to ease the read of the data for the user.
    '''
    for msg in canbus:
        if msg.arbitration_id in idlist:
            return (idlist[msg.arbitration_id], array.array('B',msg.data))

class CanReader(Node):
    '''
    The Node is created with a publisher.
    It takes a canbus as a parameter.
    The canbus is the bus that the car is connected to.
    '''

    def __init__(self, bus):
        super().__init__('can_reader')
        self.bus = bus
        self.publisher = self.create_publisher(BusCan, 'can_data', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = BusCan()
        msg.arbitration_id, msg.data = can_reader(self.bus)
        self.publisher.publish(msg)
        self.get_logger().info("Publishing :" + str(msg.arbitration_id) + " " + ", ".join(str(int(hex(b),16)) for b in msg.data))

def main():
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    rclpy.init()

    bus = can.ThreadSafeBus(interface='socketcan', channel="can0",timeout=0.01)

    can_reader = CanReader(bus)

    try:
        rclpy.spin(can_reader)
    except KeyboardInterrupt:
        pass

    bus.shutdown()
    print("done")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_reader.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
