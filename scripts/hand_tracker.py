#!/usr/bin/env python3
""" A node that tracks the hand and draws landmarks on the hand """

# Import libraries
import cv2
import mediapipe as mp
import numpy as np
import time

#Import ROS libraries and message types
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray  # Message format for the hand tracker

bridge=CvBridge()


def landmark_tracker():
    
    pub = rospy.Publisher('hand_tracker_publisher', Float32MultiArray, queue_size=20)
    # Create a publisher that sends image data from the camera
    pub_img=rospy.Publisher('rgb_img', Image, queue_size=20)
    # rospy.init_node('hand_tracker', anonymous=True)
    
    rate=rospy.Rate(25)

    while not rospy.is_shutdown():

        msg=Float32MultiArray()
        msg.data=[0,0,0]    #Initialize the message to be sent

        #Track hands and draw landmarks
        mp_hand = mp.solutions.hands
        mp_draw = mp.solutions.drawing_utils
        
        cap = cv2.VideoCapture(0)

        # Array to store the centroid of hand from continuous frames
        points = []

        with mp_hand.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8) as hands:
            

            while cap.isOpened():
                
                success, img = cap.read()

                # Preprocessing the image
                image=cv2.flip(img,1)
                img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                # Publishing raw image data
                pub_img.publish(bridge.cv2_to_imgmsg(img_rgb, "rgb8"))

                results = hands.process(img_rgb)  # Inferencing the image

                if results.multi_hand_landmarks:
                    
                    for hand_landmarks in results.multi_hand_landmarks:
                        
                        # Checking if the palm is open or closed
                        if(hand_landmarks.landmark[8].y < hand_landmarks.landmark[5].y 
                        or hand_landmarks.landmark[12].y < hand_landmarks.landmark[9].y 
                        or hand_landmarks.landmark[16].y < hand_landmarks.landmark[13].y 
                        or hand_landmarks.landmark[20].y < hand_landmarks.landmark[17].y):
                            
                            print("Palm open!")
                            msg.data=[0,0,0]
                            points.clear()
                            break

                        else:
                            print("Palm closed!")

                            x=(float(hand_landmarks.landmark[5].x) + float(hand_landmarks.landmark[9].x) + float(hand_landmarks.landmark[13].x) + float(hand_landmarks.landmark[17].x))/4
                            y=(float(hand_landmarks.landmark[5].y) + float(hand_landmarks.landmark[9].y) + float(hand_landmarks.landmark[13].y) + float(hand_landmarks.landmark[17].y))/4

                            points.append([x,y]) # Append the centroid of the hand to the list

                            #After 2 points are added to the list, calculate the angle and distance between them
                            if(len(points) == 2):
                                y_diff = (points[1][1] - points[0][1])
                                x_diff = (points[1][0] - points[0][0])
                                angle = np.arctan2(y_diff, x_diff)

                                dist=np.sqrt((y_diff*480)**2 + (x_diff*640)**2)

                                if(dist<10 or dist>150):
                                    print("No movement")
                                    points.clear()
                                else:

                                    comp_x=0.01
                                    comp_y=0.01

                                    #Making sure that the values don't exceed certain bounds
                                    if(x_diff<0):
                                        comp_x = -comp_x
                
                                    if(y_diff<0):
                                        comp_y = -comp_y
                                        
                                    msg.data=[min(x_diff,comp_x),min(y_diff,comp_y),angle]
                                    
                                    del points[0] # Delte the first element of the list

                        #Breaking to repeat the process from scratch            
                        break

                    pub.publish(msg)
                    # Uncomment to see the landmarks
                    #mp_draw.draw_landmarks(image, hand_landmarks, mp_hand.HAND_CONNECTIONS)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('hand_tracker', anonymous=True)

    try:
        landmark_tracker()
    except rospy.ROSInterruptException:
        pass
