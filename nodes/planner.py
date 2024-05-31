#!/usr/bin/python3

import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float64

# Initialize constants
BASE_VELOCITY = 5.0
MIN_VARIATION = -2.0
MAX_VARIATION = 2.0
CENTER = 320 #Center of the RAW Resolution

# PID parameters
K_P = 0.5
K_I = 0.2
K_D = 0.1

# Initialize global variables
delta_velocity = 0
previous_error = 0
integral_error = 0
start_time = time.time()

def PID(data: Float32) -> None:
    global delta_velocity
    global start_time
    global previous_error
    global integral_error

    # Get distance from center value
    cx = data.data
    dt = time.time() - start_time

    current_error = CENTER - cx

    # Proportional
    proportional_error = current_error

    # Integral
    integral_error += current_error * dt

    # Derivative
    derivative_error = (current_error - previous_error) / dt if dt > 0 else 0

    # Velocity output
    delta_velocity = min(max((K_P * proportional_error) + (K_I * integral_error) + (K_D * derivative_error), MIN_VARIATION), MAX_VARIATION)

    # Update previous error and time
    previous_error = current_error
    start_time = time.time()

    # Debugging prints
    rospy.loginfo(f"current_error: {current_error}, velocity: {delta_velocity}")

def node():
    # Initialize ROS node
    rospy.init_node('velocity', anonymous=True)

    # Subscriber
    rospy.Subscriber("image_data", Float32, PID)

    # Publishers
    pub_left = rospy.Publisher('/car/front_left_velocity_controller/command', Float64, queue_size=15)
    pub_right = rospy.Publisher('/car/front_right_velocity_controller/command', Float64, queue_size=15)

    rate = rospy.Rate(25)  # 25 Hz

    while not rospy.is_shutdown():
        # Publish velocities
        pub_left.publish(BASE_VELOCITY - delta_velocity)
        pub_right.publish(BASE_VELOCITY + delta_velocity)
        rate.sleep()

if __name__ == "__main__":
    node()
