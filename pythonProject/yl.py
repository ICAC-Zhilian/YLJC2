import cv2
import mediapipe as mp
import math
import numpy as np
import pygame
class poseDumbbell:
    def __init__(self):
        self.mpose=mp.solutions.pose
        self.pose=self.mpose.Pose()
        self.draw=mp.solutions.drawing_utils
    #获取姿态
    def finPose(self,img):
        imggray=cv2. cvtColor(img,cv2.COLOR_BGR2RGB)
        self.rs=self.pose.process(imggray)
        if self.rs.pose_landmarks:
            self.draw.draw_landmarks(img,self.rs.pose_landmarks,self.mpose.POSE_CONNECTIONS)
        return img
    #定位
    def findposition(self,img):
        self.lmlist=[]
        if self.rs.pose_landmarks:
            for id,lm in enumerate(self.rs.pose_landmarks.landmark):
                h,w,c=img.shape
                cx,cy=int(lm.x * w),int(lm.y * h)
                self.lmlist.append([id,cx,cy])
                cv2.circle(img,(cx,cy),5,(200,0,0),cv2.FILLED)
        return self.lmlist
    #计算胳膊弯曲角度
    def computeAngle(self,img,p1,p2,p3):
        x1,y1=self.lmlist[p1][1:]
        x2,y2=self.lmlist[p2][1:]
        x3,y3=self.lmlist[p3][1:]
        #手臂弯度
        angle=math.degrees(math.atan2(y3-y2,x3-x2)-math.atan2(y1-y2,x1-x2))
        if  angle<0:
            angle+=360
        cv2.putText(img,str(int(angle)),(x2-50,y2+50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,200),2)
        return  angle
detector=poseDumbbell()
#导入视频
cap=cv2.VideoCapture("YL.mp4")

dir=0
count=0
while True:
    pygame.init()
    pygame.mixer.init()
    bgsound=pygame.mixer.Sound("ok.wav")
    ok, frame = cap.read()
    if ok:
        img=cv2.resize(frame,(640,480))
        img=detector.finPose(img)
        lmlist = detector.findposition(img)
        if len(lmlist)>0:
            angle=detector.computeAngle(img,12,14,16)
            per=np.interp(angle,(210,310),(0,100))
            # print(per)
            if per==100:
                if dir==0:
                    count +=0.5
                    dir=1
                    bgsound.play()
            if per ==0:
                if dir==1:
                    count +=0.5
                    dir=0
            cv2.putText(img,str(int(count)),(45,450),cv2.FONT_HERSHEY_PLAIN,2,(0,0,200),7)

    cv2.imshow("", img)
    c = cv2.waitKey(60)
    if c == 27:
        cap.release()
        break
cv2.destroyAllWindows()