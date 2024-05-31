#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Define global variables with initial values
cx, cy = 0.0, 0.0

def ImgCallBack(data):
    global cx, cy  # Declare as global to modify the global variables
    
    # CVBRIDGE CONVERTION
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    
    # IMAGE CROPPING
    _, width, _ = cv_img.shape
    crop_img = cv_img[240:360, 1:width]

    # HSV CONVERTION
    hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    # THRESHOLD
    lower_limit = np.array([20, 100, 100])
    upper_limit = np.array([50, 255, 255])

    mask_img = cv2.inRange(hsv_img, lower_limit, upper_limit)

    # CENTROID
    res_img = cv2.bitwise_and(crop_img, crop_img, mask=mask_img)
    m = cv2.moments(mask_img, False)

    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        cx, cy = width / 2, (360 - 240) / 2
    
    print(cx, cy)

    cv2.circle(res_img, (int(cx), int(cy)), 3, (0, 0, 255), -1)

    # SHOW RESULTS
    cv_img_resized = cv2.resize(cv_img, (640, 480))
    res_img_resized = cv2.resize(res_img, (640, 480))
    cv2.imshow("RAW", cv_img_resized)
    cv2.imshow("FOLLOWER", res_img_resized)
    cv2.waitKey(1)

def node():
    # SUBSCRIBER
    rospy.Subscriber("/car/image_raw", Image, ImgCallBack)
    
    # PUBLISHER
    pub = rospy.Publisher('/image_frame', Float32, queue_size=10)
    rospy.init_node('/image_frame')
    rate = rospy.Rate(25)  # 25Hz

    global cx  # Declare as global to access the global variable
    while not rospy.is_shutdown():
        pub.publish(cx)
        rate.sleep()

if __name__ == "__main__":
    node()
