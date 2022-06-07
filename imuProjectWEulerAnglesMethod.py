#Thanh Phong Trinh
#*************************************
from turtle import *
from vpython import *
import serial
from time import *
import numpy as np
import math
arduinoData = serial.Serial('com7',115200)
sleep(1)

scene.range = 90 # create visual box to see the design
scene.forward = vector(-1,-1,-1) #create view direction
scene.width = 720
scene.height = 720

toRad = np.pi/180
toDeg = 1/toRad

myBody = cylinder( radius = 4, length = 100, color = color.white, pos = vector(-50,0,0))
myCell = ellipsoid(length = 25, height = 8, width = 8, color = color.yellow, pos = vector(-49.5, 2, 0))
myHead = ellipsoid(length = 40, height= 8.3, width=8, color= color.white, pos = vector(-50,0,0))
myTail = cone(radius = 4, length = 8, color = color.white, pos = vector(50,0,0))
myWing = box(color = color.white, size = vector(85,1,39), axis = vector(0,0,1), pos = vector(-27,0,0))
myCotrolWing1 = box(color = color.white, size = vector(45,1,15), axis = vector(0,0,1), pos = vector(42,0,0)) 
myControlWing2 = box(color = color.white,size = vector(10,15,1), axis = vector(0,1,0), pos = vector(42,9,0))
Xarrow = arrow(length = 25, shaftwidth = 1, color = color.red, axis= vector(1,0,0))
Yarrow = arrow(length = 25, shaftwidth = 1, color = color.green, axis= vector(0,1,0))
Yarrow = arrow(length = 25, shaftwidth = 1, color = color.blue, axis= vector(0,0,1))
myAirPlane = compound([myBody,myCell,myHead,myTail,myWing,myCotrolWing1,myControlWing2])

while(True):
    while(arduinoData.inWaiting() ==0):
        pass
    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(',')
    pitch = float(splitPacket[4])*toRad + np.pi # Plus Pi to adjust airplane corrdidate same Sensor cood. It depends on the way you set your data from arduino
    roll = - float(splitPacket[5])*toRad #adding 'minus'  to adjust airplane corrdidate same Sensor cood. It depends on the way you set your data from arduino 
    yaw = float(splitPacket[6])*toRad + np.pi # Plus Pi to adjust airplane corrdidate same Sensor cood. It depends on the way you set your data from arduino
    direcVectorK = vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    unitVectorJ = vector(0,1,0)
    direcVectorI = cross(direcVectorK,unitVectorJ)
    direcVectorJ = cross(direcVectorI,direcVectorK)
    rotaVectorJ = direcVectorJ*cos(roll) + cross(direcVectorK,direcVectorJ)*sin(roll) # using Rodrigues' rotation formula
    myAirPlane.axis = direcVectorK
    myAirPlane.up = rotaVectorJ


