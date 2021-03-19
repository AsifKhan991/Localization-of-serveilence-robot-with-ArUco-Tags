import numpy as np
import serial
import cv2
import cv2.aruco as aruco
import time

COM='COM7'
baud=57600

out = cv2.VideoWriter('Augmented.avi',  cv2.VideoWriter_fourcc(*'MJPG'), 30, (640,480))

port=1
basespeed=90
stimer=0
stat=0
try:
    arduino = serial.Serial(COM, baud)
    print(' Port found! ')
except serial.serialutil.SerialException:
    print(' Port not found! ')
    port=0

def search():
    delay=2000000
    global stat
    global stimer
    if cv2.getTickCount()-stimer>delay:
        if stat==0:
            drive(0,basespeed+20)
            stat=1
        else:
            drive(0,0)
            stat=0
        stimer=cv2.getTickCount()


def drive(left,right):
    if left>basespeed:
        left=basespeed
    if left<0:
        left=0
    if right>basespeed:
        right=basespeed
    if right<0:
        right=0
        
    if port == 1:
        data=str(left)+","+str(right)+";\n"
        arduino.write(data.encode())

        
cap = cv2.VideoCapture(0)
fps=0
timer=0
i=1
while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
    if corners:
        for corner,id in zip(corners, ids):
            if( id==i) :
                timer = cv2.getTickCount()
                frame = aruco.drawDetectedMarkers(frame, [corner])
                x=int((corner[0][0][0]+corner[0][2][0]))//2
                if abs(corner[0][0][0]-corner[0][2][0])>200:
                    i=i+1
                cv2.putText(frame,"Marker"+str(id)+"found",(x-70,240),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                diff=320-x
                if(diff>0):
                    drive(basespeed-diff,basespeed)
                else:
                    drive(basespeed,basespeed+diff)
                print(i)
                break
            if(cv2.getTickCount() - timer>10000000):
                search()
                cv2.putText(frame, "searching..", (240,240), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0), 2)
            else:
                drive(0,0)
    else:
        if(cv2.getTickCount() - timer>10000000):
            search()
            cv2.putText(frame, "searching..", (240,240), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0), 2)
        else:
            drive(0,0)
        

         
    cv2.imshow('Display', frame)
    out.write(frame)


    if i==4:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
drive(0,0)
if port == 1:
    arduino.close()
cap.release()
cv2.destroyAllWindows()
