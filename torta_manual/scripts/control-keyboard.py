#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import numpy
import math
import Model
from std_msgs.msg import Float32MultiArray

# a set of strings of all currently-pressed keys
pressed_keys = set()

'''
MANUAL USAGE to control the robot.

for movement:
        W -> Forward
    A   S   D -> Right
    |    \ 
    v     v
   Left  Backward
Q: for clockwise turn
E: for anti-clockwise turn

hold L-SHIFT to speed up (limit= MAX_VEL)
hold L-CTRL to speed down (limit= 0)
# when releasing shift/ctrl key, the gain speed will be reset.

Press ESC to turn off the program
'''

# initializing rospy nodes and topics' publisher
rospy.init_node('keyboard_robot_control', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
angular_publisher = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

# this array will be used to send the kinematic's model data to the robot STM
arr = Float32MultiArray()

# twist_cmd is storing x,y,z linear and angular velocities 
twist_cmd = Twist()

# initializing speed and kinematic model variables
xlinearVelocity  = 0.0           # X position represents Forward and Backwards velocity
ylinearVelocity  = 0.0         
maxAngularVelocity = 0.585     # radian per second
angularVelocity = 0.0
wheel_radius = 0.075
lx = 22  #horizontal distance from the center of the robot
ly = 15.8  #vertical distance from the center of the robot
model_output= numpy.zeros(4)

# You can tune the following constants to your best fit
# declaring constants of speed in all directions:
VERTICAL=4.0          #'w','s'
HORIZONTAL=4.0        #'d','a'
CLOCKWISE=0.1       # Q
ANTICLOCKWISE=-0.1  # E

gain=float(1)# initializing variables
xlinearVelocity  = 0           # X position represents Forward and Backwards velocity
ylinearVelocity  = 0         
maxAngularVelocity = 0.585     # radian per second
angularVelocity = 0
wheel_radius = 0.075
lx = 22  #horizontal distance from the center of the robot
ly = 15.8  #vertical distance from the center of the robot
model_output= numpy.zeros(4)
MAX_GAIN = 3


# function called on press of the key
def on_press(key):
    #globalizing variables
    global xlinearVelocity, ylinearVelocity, angularVelocity,model_output, lx, ly, gain, pressed_keys
    # add the currently pressed key
    # if it's alphanumeric:
    if hasattr(key, 'char'):
        pressed_keys.add(str(key.char).lower()) # the character is in lowercase to make (i.e: 'A'/'a') the same
    else:
        # For special keys, adding their name to the list
        pressed_keys.add(str(key)) ## for example: shift will be 'Key.shift'
    
    seeKeyboard() # check the pressed keys
    # store the x,y,z of linear velocity and angular velocity
    twist_cmd.linear.x = xlinearVelocity  
    twist_cmd.linear.y = ylinearVelocity
    twist_cmd.angular.z = angularVelocity
    # Applying kinematic Model to produce velocities on each wheel
    model_output = Model.kinematicModel(xlinearVelocity, ylinearVelocity, angularVelocity, wheel_radius, lx, ly).mecanum_4_vel()
    # publish the results to see speeds
    pub.publish(twist_cmd)
    arr.data = model_output
    # publish the model data to the robot
    angular_publisher.publish(arr)

# function called on release of key 
def on_release(key):
    global xlinearVelocity, ylinearVelocity, angularVelocity,model_output, lx, ly, gain, pressed_keys
    # remove the key released
    if hasattr(key, 'char'): #alphanumeric character
        try:
            pressed_keys.remove(str(key.char).lower())
        except (ValueError, KeyError) as e:
            print(key)
            print(str(key.char))
            # the released shift key is acting as a NoneObject  with code <<66032>>
            if ('Key.shift' in pressed_keys):
                # when releasing shift key, the gain speed will be reset.
                pressed_keys.remove('Key.shift')
                gain = 1
            pass
    else: # special character
        if (str(key) in pressed_keys):
            pressed_keys.remove(str(key))
    
    if (not hasattr(key, 'char')): # if it's a special character released
        print("Special Character released")
    if(key==keyboard.Key.esc):
        exit() # Exit the program when ESC pressed
    if (key==keyboard.Key.shift):
        # when releasing shift key, the gain speed will be reset.
        if ('Key.shift' in pressed_keys):
            pressed_keys.remove('Key.shift')
        gain = 1
    if (key==keyboard.Key.ctrl): # CTRL released
        gain = 1
        
    seeKeyboard() # check the pressed keys


    # store the x,y,z of linear velocity and angular velocity
    twist_cmd.linear.x = xlinearVelocity 
    twist_cmd.linear.y = ylinearVelocity
    twist_cmd.angular.z = angularVelocity
    # Applying kinematic Model to produce velocities on each wheel
    model_output = Model.kinematicModel(xlinearVelocity, ylinearVelocity, angularVelocity, wheel_radius, lx, ly).mecanum_4_vel()
    # publish the results to see speed
    pub.publish(twist_cmd)
    arr.data = model_output
    # publish the model data to the robot
    angular_publisher.publish(arr)

# function checks for pressed keys
def seeKeyboard():
    global gain,xlinearVelocity,ylinearVelocity,angularVelocity,pressed_keys
    xlinear=0
    ylinear=0
    angular=0
    if 'Key.esc' in pressed_keys:
        exit() # Exit the program when ESC pressed
    if 'Key.shift' in pressed_keys:
        if (gain + 0.1 <= float(MAX_GAIN)): # limit the speed gain = MAX_GAIN
            gain += 0.1
    if 'Key.ctrl' in pressed_keys:
        if (gain-0.1>=0): # limit = zero
            gain -= 0.1
    if 'd' in pressed_keys:
        #ylinearVelocity=VERTICAL*gain
        ylinear+=VERTICAL*gain

    if 'w' in pressed_keys:
        #xlinearVelocity=HORIZONTAL*gain
        xlinear+=HORIZONTAL*gain


    if 'a' in pressed_keys:
        #ylinearVelocity=-VERTICAL*gain
        ylinear-=VERTICAL*gain

    if 's' in pressed_keys:
        #xlinearVelocity=-HORIZONTAL*gain
        xlinear-=HORIZONTAL*gain

    if 'q' in pressed_keys:
        #angularVelocity=ANTICLOCKWISE*gain
        angular+=ANTICLOCKWISE*gain

    if 'e' in pressed_keys:
        #angularVelocity=CLOCKWISE*gain
        angular+=CLOCKWISE*gain
    angularVelocity=angular
    xlinearVelocity=xlinear
    ylinearVelocity=ylinear

def control_robot():
    while not rospy.is_shutdown():
        with keyboard.Listener( on_press=on_press, on_release=on_release) as listener: 
            listener.join()
        

if __name__ == '__main__':
    control_robot()