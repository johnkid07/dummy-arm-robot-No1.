
import rospy
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import math
import numpy as np
def pub_msg():
    pub = rospy.Publisher('detecXY', Twist,queue_size=10)


    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(10)

    cam = cv2.VideoCapture(0)
    mpHand = mp.solutions.hands
    hand = mpHand.Hands(max_num_hands=1)
    mpDraw = mp.solutions.drawing_utils 
    con = Twist()

    while not rospy.is_shutdown():

        ret, frame = cam.read()
        if ret == True:
            frame = cv2.flip(frame, 1)


            imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hand.process(imgRGB)
       

         
            lmList = []

            if result.multi_hand_landmarks:
                for handLMS in result.multi_hand_landmarks:
                    mpDraw.draw_landmarks(frame, handLMS,mpHand.HAND_CONNECTIONS)
                    for id, lm in enumerate(handLMS.landmark):
                        if id == 9:
                            h, w, c = frame.shape
                            cx, cy = int(lm.x*w), int(lm.y*h)
                            
                            
                            con.linear.x = cx
                            con.linear.y = cy

                            
                            
                            rospy.loginfo(con)
                            pub.publish(con)
                            rate.sleep()
            if result.multi_hand_landmarks:
                for handLMS in result.multi_hand_landmarks:
                    myHand  = result.multi_hand_landmarks[0]
                    for id, lm in enumerate(myHand.landmark):
                        h, w, c = frame.shape
                        cx, cy = int(lm.x*w), int(lm.y*h)
                        lmList.append([id, cx, cy])
                        

                    if len(lmList) != 0:
                        x1, y1 = lmList[4][1],  lmList[4][2]
                        x2, y2 = lmList[8][1], lmList[8][2] 
                        cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),3)
                        length = math.hypot(x2-x1,y2-y1)
                        if length < 50:
                             cv2.line(frame,(x1,y1),(x2,y2),(0,0,0),3)
                        Pos = np.interp(length, [50, 220], [0, 100])
                        Posgripper=  (round(Pos))
                   
                        converted_Posgripper = str(Posgripper)
                        cv2.putText(frame, str(Posgripper), (50, 60), cv2.FONT_HERSHEY_COMPLEX, 2,  (255, 0, 0))
                     
                        Servopos=(100-Posgripper)
                        if Servopos >= 0:
                            Servopos = 684 - (Posgripper*2 )
                        con.angular.x = Servopos




                



            cv2.imshow('result',frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
if __name__ == '__main__':
    try:
        pub_msg()
    except rospy.ROSInterruptException:
        pass

