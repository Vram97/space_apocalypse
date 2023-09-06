#!/usr/bin/env python3
""" A node that tracks the hand and draws landmarks on the hand """

# Import libraries
import cv2
import mediapipe as mp
import numpy as np
import time

#Import ROS libraries and message types
import rospy
from std_msgs.msg import Float32MultiArray

print("Before function")


def landmark_tracker():
    print("Inside function")
    pub = rospy.Publisher('hand_tracker_publisher', Float32MultiArray, queue_size=20)

    rospy.init_node('hand_tracker', anonymous=True)
    
    rate=rospy.Rate(25)

    while not rospy.is_shutdown():
        print("Inside while loop -ROS")

        msg=Float32MultiArray()
        msg.data=[0,0,0]

        #Track hands and draw landmarks
        
        mp_hand = mp.solutions.hands
        mp_draw = mp.solutions.drawing_utils

        cap = cv2.VideoCapture(0)
        print("Video cap")
        points = []

        with mp_hand.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8) as hands:
            # print("with hands")
            while cap.isOpened():
                # print("yo")
                success, img = cap.read()
                # msg.data=[0,0,0]
                # start = time.time()
                image=cv2.flip(img,1)
                img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(img_rgb)
                # end = time.time()
                # print(end - start)
                if results.multi_hand_landmarks:
                    # print("if results")
                    for hand_landmarks in results.multi_hand_landmarks:
                        # print("for hand_landmarks")
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

                            points.append([x,y])

                            if(len(points) == 2):
                                y_diff = (points[1][1] - points[0][1])
                                x_diff = (points[1][0] - points[0][0])
                                angle = np.arctan2(y_diff, x_diff)

                                dist=np.sqrt((y_diff*480)**2 + (x_diff*640)**2)

                                if(dist<10 or dist>150):
                                    print("No movement")
                                    points.clear()
                                else:
                                    # print("Angle: ", angle)
                                    # print("Distance: ", dist)
                                    comp_x=0.01
                                    comp_y=0.01

                                    if(x_diff<0):
                                        comp_x = -comp_x
                
                                    if(y_diff<0):
                                        comp_y = -comp_y
                                        
                                    msg.data=[min(x_diff,comp_x),min(y_diff,comp_y),angle]
                                    cv2.line(image, (int(points[0][0]*640), int(points[0][1]*480)), (int(points[1][0]*640), int(points[1][1]*480)), (0,255,0), 5)
                                    del points[0] # Delte the first element of the list
                                    # pub.publish(msg)

                        mp_draw.draw_landmarks(image, hand_landmarks, mp_hand.HAND_CONNECTIONS)
                        break
                    
                # cv2.imshow("Image", image)
                    pub.publish(msg)
                    rate.sleep()
                # pub.publish(msg)
            # pub.publish(msg)
        # pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        landmark_tracker()
    except rospy.ROSInterruptException:
        print("Issue")
        pass
