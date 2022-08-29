import socket
from tkinter.tix import Tree
import pygame
import time
import pickle

def remap(x, in_min, in_max, out_min, out_max):
    '''
    Small and simple function used to remap the data from to joystick to a desired format.
    '''
    
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joystick_input(joystick, direction,speed, steering, brake, current_y, current_mode):
    '''
    This function retrieves data from the joystick and convert them.
    It will update the current motion data for the vehicle such as speed, steering brake mode...
    In order to make the driving smoother, the steering angle will take speed into account

    '''

    # Define the reachable angle value for different modes
    mode_1 = [0,7,16,25,37,1]
    mode_2 = [0,6,13,20,30,2]
    mode_3 = [0,5,10,17,25,3]

    brake = 0

    go = True # variable for debouncing

    pygame.event.pump()

    y = joystick.get_axis(0) # steering
    x = joystick.get_axis(1) # speed

    # update speed value
    speed = remap(-x,-1,1,-191,190)

    # choose a mode that will adjust angle according to speed
    sp = abs(speed)
    if sp <= 65 :
        mode = mode_1
    elif sp > 65 and sp <= 130:
        mode = mode_2
    elif sp > 130:
        mode = mode_3

    # Debounce
    if current_y < y + 0.03 and current_y > y - 0.03 and current_mode == mode[5]:
        go = False
    else:
        current_y = y

    # Update brake value
    if joystick.get_button(10):
        brake = 1
    if joystick.get_button(11):
        brake = 2
    
    if y < 0.05 and y > -0.05:
        y = 0
    
    if x < 0.05 and x > -0.05:
        x = 0

    # Update steering value
    if go == True :
        if y < 0.05 and y > -0.05:
            steering = 0
        
        elif y < 0.25 and y >= 0.05 :
            steering = remap(y*y, 0.05, 0.25, mode[0], mode[1])
        
        elif y < 0.5 and y >= 0.25:
            steering = remap(y,0.25,0.5, mode[1], mode[2])
        
        elif y < 0.75 and y >= 0.5 :
            steering = remap(y,0.5,0.75,mode[2],mode[3])
        
        elif y <= 1 and y >= .75 :
            steering = remap(y,0.75,1, mode[3],mode[4])

        elif y <= -0.05 and y > -0.25:
            steering = remap(y, -0.05, -0.25, mode[0], -mode[1])
        
        elif y <= -0.25 and y > -0.5:
            steering = remap(y, -0.25, -0.5, -mode[1], -mode[2])
        
        elif y <= -0.5 and y > -0.75:
            steering = remap(y,-0.5,-0.75,-mode[2],-mode[3])
        
        elif y <= -0.75 and y >= -1:
            steering = remap(y, -0.75,-1,-mode[3],-mode[4])

    # Update direction value according to speed value    
    if speed < -1:
        direction = 1
        speed = -speed
    else :
        direction = 0
    
    current_mode = mode[5]

    return(direction,int(speed),int(steering),brake, current_y, current_mode)

class variables :
    '''
    Define a class that will take vehicle's motion variables.
    Those variables are then updated according to joystick's position.
    '''

    def __init__(self):
        self.mode = 0
        self.direction = 0
        self.speed = 0
        self.steering = 0
        self.brake = 0
        self.current_y = 0
        self.current_mode = 1
    
    def get_data(self):
        '''
        This will return the data currently contained inside an instance of the variables class
        '''
        data = [self.mode, self.direction, self.speed, self.steering,  self.brake]
        print(data)
        return data
        

def main():
    '''
    This function create a client that will connect to a server.
    Once connected, the client will sent the motion variables to the server.
    The former will be able to handle those variables through a ROS2 node.
    This script was designed due to the incompatibility between two ROS2 distro (foxy and dashing)
    '''

    PORT = 4852
    ADDRESS = "192.168.9.2"

    var = variables()

    my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    my_socket.connect((ADDRESS,PORT))

    pygame.init()

    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joysticks found.")

    joystick = pygame.joystick.Joystick(0)

    joystick.init()

    while True :
        
        var.direction,var.speed, var.steering, var.brake, var.current_y, var.current_mode = joystick_input(joystick, var.direction, var.speed, var.steering, var.brake, var.current_y, var.current_mode)

        my_socket.send(pickle.dumps(var.get_data()))
        
        print(var.get_data)

        time.sleep(0.05)



if __name__ == "__main__" :
    main()