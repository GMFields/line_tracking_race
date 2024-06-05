#!/usr/bin/python3

import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float64

# Initialize constants
BASE_VELOCITY = 5.0
CENTER = 320 #Center of CAMERA 

# PID parameters
K_P = 0.01 #0.01
K_D = 0.005 #0.005
K_I = 0.001 #0.001

# Initialize global variables
velocity_var = 0
previous_error = 0
integral_error = 0
start_time = time.time()

def pid_controller(data: Float32) -> None:
    global velocity_var
    global start_time
    global previous_error
    global integral_error

    # Obtain real value passed by the handler 
    cx = data.data
    # Compute new delta time 
    dt = time.time() - start_time

    # Current iteration error
    current_error = CENTER - cx

    # Current proportional error
    proportional_error = current_error

    # Current integral error
    integral_error += current_error * dt

    # Current derivative error
    derivative_error = (current_error - previous_error) / dt if dt > 0 else 0

    # Value of variation of velocity
    velocity_var = (K_P * proportional_error) + (K_I * integral_error) + (K_D * derivative_error)
    rospy.loginfo(f"vel: {velocity_var}")

    # Update previous error and time
    previous_error = current_error
    start_time = time.time()

    # Debugging prints
    rospy.loginfo(f"current_error: {current_error}, velocity: {velocity_var}")

def node() -> None:
    # Initialize node
    rospy.init_node('velocity', anonymous=True)

    # Subscribe, Receive and Handle value delivered by image_handler 
    rospy.Subscriber("/image_data", Float32, pid_controller)

    # Initialize publisher to update velocity controllers
    pub_left = rospy.Publisher('/car/front_left_velocity_controller/command', Float64, queue_size=15)
    pub_right = rospy.Publisher('/car/front_right_velocity_controller/command', Float64, queue_size=15)
    rate = rospy.Rate(25)  # 25 Hz

    while not rospy.is_shutdown():
        # Publish velocities
        pub_left.publish(BASE_VELOCITY - velocity_var)
        pub_right.publish(BASE_VELOCITY + velocity_var)
        rate.sleep()

if __name__ == "__main__":
    node()
