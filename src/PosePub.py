#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

def main():
    rospy.init_node('cam')
    rate = rospy.Rate(10) # 1hz

    odom_pub = rospy.Publisher('/odom2', Pose2D, queue_size=50)
    odom_msgs = Pose2D()


    while not rospy.is_shutdown():

        ret, frame = cap.read()
        frame = cv2.resize(frame, dsize=(480,480))
        frame = cv2.medianBlur(frame, 9)
        x_pixel = 0.0095
        y_pixel = 0.00471
    
        x_base = 125
        y_base = 7
    
        red_x = 0
        red_y = 0
        blue_x = 0
        blue_y = 0
    
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #red = cv2.inRange(hsv, (0,)
        lower_red = cv2.inRange(hsv, (0, 100, 100), (5, 255, 255))  # 범위내의 픽셀들은 흰색, 나머지는 검은색
        upper_red = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        red = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)

        blue = cv2.inRange(hsv, (100, 110, 100),(110, 255, 255))

        red_image = cv2.bitwise_and(frame, frame, mask = red)
        blue_image = cv2.bitwise_and(frame, frame, mask = blue)
    
        cnt = 0
        sum_x = 0
        sum_y = 0
    
        for row in range(480):
            for col in range(480):        
                if(red_image[row][col][0] != 0):
                    sum_y = sum_y + row
                    sum_x = sum_x + col
                    cnt = cnt + 1
        if(cnt != 0):
            red_x = sum_x / cnt
            red_y = sum_y / cnt 

        cnt = 0
        sum_x = 0
        sum_y = 0
    
        for row in range(480):
            for col in range(480):
                if(blue_image[row][col][0] != 0):
                    sum_y = sum_y + row
                    sum_x = sum_x + col
                    cnt = cnt + 1
        if(cnt != 0):
            blue_x = sum_x / cnt
            blue_y = sum_y / cnt

        red_x = (red_x - x_base) * x_pixel
        red_y = (red_y - y_base) * y_pixel
    
        blue_x = (blue_x - x_base) * x_pixel
        blue_y = (blue_y - y_base) * y_pixel
        
        x = ((red_x + blue_x)/2) 
        y = ((red_y + blue_y)/2)
        theta = 0
        if((red_x - blue_x) != 0 and (red_y - blue_y) != 0):
            if(red_x > blue_x):
                theta = (red_y - blue_y) / (red_x - blue_x)
                theta = np.arctan2((red_y - blue_y), (red_x - blue_x))
            else:
                theta = np.arctan2((red_y - blue_y), (red_x - blue_x))

                ##theta = -1 * theta
        
        print("x: ", x)
        print("y: ", y)
        print("theta: ", theta)
        odom_msgs.x = x
        odom_msgs.y = y
        odom_msgs.theta = theta
        odom_pub.publish(odom_msgs)
    
        cv2.imshow('frame', red_image)
        cv2.imshow('fram', blue_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()


if __name__ == '__main__':
    try:
        cap = cv2.VideoCapture(2)
        main()
    except rospy.ROSInterruptException:
        pass