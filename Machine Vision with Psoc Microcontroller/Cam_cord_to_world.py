import numpy as np
import serial
import cv2
import time
import math

cap=cv2.VideoCapture(0)
ser=serial.Serial()
ser.baudrae=9600
ser.port='COM3'
link1=5.842
link2=5.842
dd = 2;

def theta(x,y):
    link1=5.842
    link2=5.842
    T2= -(math.degrees(math.acos((x**2 + y**2 - link1**2 -link2**2)/(2*link1*link2))))
    check = math.sin(math.radians(T2))
    heck = math.cos(math.radians(T2))                 
    T1=math.degrees(math.atan(float(y)/x))+ math.degrees(math.atan(float((link2*check))/(link1+ link2*heck)))
    T1 = int(round(T1))
    T2 = int(round(T2))
    print(T1,T2)
    return [T1,T2]

cm_to_pixelx=19/640.0 #parallel surface MINE IS 19 CM OKAY COOL
cm_to_pixely=14.6/480.0

R180_X=[[1,0,0],[0,np.cos(np.pi),-np.sin(np.pi)],[0,np.sin(np.pi),np.cos(np.pi)]]
Rad=(-1/180)*np.pi
RZ=[[np.cos(Rad),-np.sin(Rad),0],[np.sin(Rad),np.cos(Rad),0],[0,0,1]]
#R0_C=np.dot(R180_X,RZ)
d0_C=[[-5],[11.6],[0]]
#increasing and decreasing does what
#
#1 is the axis we wanna cocatenatr the rows
H0_C=np.concatenate((R180_X,d0_C),1)
H0_C=np.concatenate((H0_C,[[0,0,0,1]]),0)


while(1):
    _,frame=cap.read()

    #convert to greyscale
    gray_image1=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cv2.imshow('background', gray_image1)    
    
    k=cv2.waitKey(5)
    if k==27:
        break
time.sleep(8)
while(1):
    _,frame=cap.read()

    #convert to greyscale
    gray_image2=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    cv2.imshow('foreground', gray_image2)
    
    Difference= np.absolute(np.matrix(np.int16(gray_image1))-np.matrix(np.int16(gray_image2)))
    Difference[Difference > 255] = 255
    Difference = np.uint8(Difference)

    cv2.imshow('difference',Difference)

    BW=Difference
    BW[BW<=100]=0
    BW[BW>100]>1

    column_sums=np.matrix(np.sum(BW,0))
    column_numbers=np.matrix(np.arange(640))
    column_mult=np.multiply(column_sums,column_numbers)
    total=np.sum(column_mult)
    total_total=np.sum(np.sum(BW))
    column_location=total/total_total

    X_location=column_location*cm_to_pixelx

    row_sums=np.matrix(np.sum(BW,1)) #change to one for rows
    row_sums=row_sums.transpose()
    row_numbers=np.matrix(np.arange(480))
    row_mult=np.multiply(row_sums,row_numbers)
    total=np.sum(row_mult)
    total_total=np.sum(np.sum(BW))
    row_location=total/total_total

    Y_location=row_location*cm_to_pixely


    #print(column_location,row_location)
    #print(X_location,Y_location)
    #if x > 0:

    PC=[[X_location],[Y_location],[0],[1]]
    P0=np.dot(H0_C,PC)

    X_0=P0[0]
    Y_0=P0[1]
    
    print(X_0,Y_0)
    try:
        [T11,T22]=theta(X_0,Y_0)
        ser.open()
        i=bytearray([T11])
        ser.write(i)
        time.sleep(1)
        j=bytearray([-T22]) 
        ser.write(j)
        time.sleep(1)
        ser.close() 
        
    except:
        r=3+5;
        print(r)

    time.sleep(1)
    
    k=cv2.waitKey(5) #this is me pressing escape
    if k==27:
        break
    
    
cv2.destroyAllWindows()
ser.close()

#print(frame)

