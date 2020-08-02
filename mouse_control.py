import pyautogui
import numpy as np
import pyfirmata
import time
import math 
import cv2
import sys


cap=cv2.VideoCapture(1)

## motor
board = pyfirmata.ArduinoMega('COM3')
it = pyfirmata.util.Iterator(board)
it.start()
# initial positions of the servos
servo1 = board.servo_config(11, 544,2400,90) 
servo2 = board.servo_config(9, 544,2400,90)
servo3 = board.servo_config(7, 544,2400,130)
servo1 = board.get_pin('p:11:o')
servo2 = board.get_pin('p:9:o')
servo3 = board.get_pin('p:7:o')
# lengths
a1=0.05630 
a2=0.043
a3=0.135
a4=0.14896
time.sleep(2)
cm_to_pixel=39/1365
cm_to_pixel2=21/767     #21/767
d1=39.5
d2=-21.5
d3=19.3
d0_C=[[d2],[d1],[d3]]
R1=[[1,0,0],[0,np.cos(np.pi),-np.sin(np.pi)],[0,np.sin(np.pi),np.cos(np.pi)]]
TT=-0.5*np.pi
##TT=np.pi
R2=[[np.cos(TT),-np.sin(TT),0],[np.sin(TT),np.cos(TT),0],[0,0,1]]
R0_C=np.dot(R1,R2)
H0_C=np.concatenate((R0_C,d0_C),1)
H0_C=np.concatenate((H0_C,[[0,0,0,1]]),0)
print(np.matrix(H0_C))





while(1):
    x,y=pyautogui.position()
    print(x,y)
    column_location=x
    row_location=y

    
    Y_location=column_location*cm_to_pixel
    Z_location=row_location*cm_to_pixel2

    PC=[[d1],[Y_location],[Z_location],[1]]
    PO=np.matrix(np.dot(H0_C,PC))
    X0=PO[0]/100
    Z0=PO[2]/100

    X0=np.nan_to_num(X0)
    Z0=np.nan_to_num(Z0)


    print(X0,Z0)
    pos=[X0,a2+a3, Z0] # define in m
    x=pos[0]
    y=pos[1]
    z=pos[2]

    phi1=np.arctan(((z-a2))/y)
    r=math.sqrt(y*y+(z-a2)*(z-a2))
##    print(r)
    phi2=np.arccos((a4*a4-a3*a3-r*r)/(-2*a3*r))
    phi3=np.arccos((r*r-a3*a3-a4*a4)/(-2*a3*a4))
    T_1=-np.arctan(x/y)
    T_2=-(0.5*np.pi-(phi1+phi2))
    T_3=(np.pi-(phi1+phi2+phi3))+T_2

    T1=T_1
    T2=T_2
    T3=T_3



    # Rotation matrices
    R0_1=[[-np.sin(T1),0,np.cos(T1)],[np.cos(T1),0,np.sin(T1)],[0,1,0]]
    ##print(np.matrix(R0_1))
    R1_2=[[np.sin(T2),-np.cos(T2),0],[-np.cos(T2),-np.sin(T2),0],[0,0,-1]]
    R2_3=[[np.sin(T3),np.cos(T3),0],[-np.cos(T3),np.sin(T3),0],[0,0,1]]
    # Displacement matrices
    d0_1=[[0],[0],[a2]]
    d1_2=[[-a3*np.sin(T2)],[a3*np.cos(T2)],[0]]
    d2_3=[[a4*np.sin(T3)],[-a4*np.cos(T3)],[0]]
    # Homogenouse transformations
    H0_1=np.concatenate((R0_1,d0_1), 1)
    H0_1=np.concatenate((H0_1, [[0,0,0,1]]), 0)
    ##print(np.matrix(H0_1))
    H1_2=np.concatenate((R1_2,d1_2), 1)
    H1_2=np.concatenate((H1_2, [[0,0,0,1]]), 0)
    ##print(np.matrix(H1_2))
    H2_3=np.concatenate((R2_3,d2_3),1)
    H2_3=np.concatenate((H2_3, [[0,0,0,1]]),0)
    ##print(np.matrix(H2_3))
    # this matrix show the final position of the end point
    H0_2=np.dot(H0_1,H1_2)
    H0_3=np.dot(H0_2,H2_3)
    ##     print(H0_3)
    ##T1=round(T_1*(180/np.pi))
    ##T2=round(T_2*(180/np.pi))
    ##T3=round(T_3*(180/np.pi))
    T1=T_1*(180/np.pi)
    T2=T_2*(180/np.pi)
    T3=T_3*(180/np.pi)
    ##     print('Angles calculated')
    ##     print(T1)
    ##     print(T2)
    ##     print(T3) 

    ## Angles to servos
    T11=-T1
    T1w=90+T11  # Theta 1 angle in degrees 80 to 140
    T2w=-T2+90  # Theta 2 angle in degrees 50 to 140. 130 is th straigth angle
    T3w=-T3+130+T2

    ##
    servo1.write(T1w)
    servo2.write(T2w)
    servo3.write(T3w)    



    k=cv2.waitKey(5)
    if k==27:
        break
