# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 03:48:28 2021

@author: juan_
"""

import sim
import math
import numpy as np
import keyboard
import sys
import time
import matplotlib.pyplot as plt

#funcion necesaria para establecer conexion con coppeliasim
def connect(port):
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print("conectado a", port)
    else: print("no se pudo conectar")
    return clientID

def clear_sensor(array):
    for i in range(len(array)):
        if array[i] > 3:
            array[i] = 0
    return array

def sense_obstacles(pos, carAngle, lidarData):
    xdata=[]
    ydata=[]
    i = 0
    x1, y1,_ = pos
    for angle in np.linspace(0,2*math.pi,len(lidarData),False):
        if (lidarData[i] > 0):
            x2, y2 = (x1 - lidarData[i]*math.cos(angle+carAngle), y1 - lidarData[i]*math.sin(angle+carAngle))
        else:
            x2, y2 = (0,0)
        xdata.append(x2)
        ydata.append(y2)
        i += 1
    return xdata, ydata
         

  
#establecemos la conexion con coppeliasim
clientID = connect(19999)
#obtener el handler para el car-like
returnCode, car_like = sim.simxGetObjectHandle(clientID, 'Manta', sim.simx_opmode_blocking)

returnCode, steer_handle = sim.simxGetObjectHandle(clientID,'steer_joint', sim.simx_opmode_blocking)
returnCode, motor_handle = sim.simxGetObjectHandle(clientID,'motor_joint', sim.simx_opmode_blocking)
returnCode, fl_brake_handle = sim.simxGetObjectHandle(clientID,'fl_brake_joint', sim.simx_opmode_blocking)
returnCode, fr_brake_handle = sim.simxGetObjectHandle(clientID,'fr_brake_joint', sim.simx_opmode_blocking)
returnCode, bl_brake_handle = sim.simxGetObjectHandle(clientID,'bl_brake_joint', sim.simx_opmode_blocking)
returnCode, br_brake_handle = sim.simxGetObjectHandle(clientID,'br_brake_joint', sim.simx_opmode_blocking)

max_steer_angle=0.5235987
motor_torque=60
dVel=1
dSteer=0.1
steer_angle=0
motor_velocity=0#dVel*10
brake_force=0

gyro_angle = 0.0

returnCode, manta_handler = sim.simxGetObjectHandle(clientID,'Manta', sim.simx_opmode_blocking) #car like manta
returnCode, gps_handler = sim.simxGetObjectHandle(clientID,'GPS', sim.simx_opmode_blocking) #gps
returnCode, gyro_handler = sim.simxGetObjectHandle(clientID,'GyroSensor', sim.simx_opmode_blocking) #gps
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_streaming)# sensor
errorCode, gyroData = sim.simxReadStringStream(clientID, 'gyro data', sim.simx_opmode_streaming)# acelerometro
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
# Obtenga datos v??lidos
time.sleep(0.1)
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
ranges = sim.simxUnpackFloats(ranges)

plt.ion()
figure, ax = plt.subplots(figsize=(10,10))
plt.ylim(-10,10)
plt.xlim(-10,10)
#line1, = ax.plot(range(len(ranges)),  ranges )
line1, = ax.plot(range(len(ranges)),range(len(ranges)))

init = True
angle = 0
delta_angle = 0

while(1):  # making a loop

    time.sleep(0.1)    
    #try:  # used try so that if user pressed other than the given key error will not be shown
    if(keyboard.is_pressed('w')):  # if key 'q' is pressed 
        if (motor_velocity<dVel*9.99):
            motor_velocity=motor_velocity+dVel
    if keyboard.is_pressed('s'):
        if (motor_velocity>-dVel*4.99):
            motor_velocity=motor_velocity-dVel
        else:
            brake_force=100
    if(keyboard.is_pressed('a')):
        if (steer_angle<dSteer*4.99):
            steer_angle=steer_angle+dSteer
    if( keyboard.is_pressed('d')):
        if (steer_angle>-dSteer*4.99):
            steer_angle=steer_angle-dSteer
    if(keyboard.is_pressed(' ')):# freno
        motor_velocity = 0
    else:
        if(np.abs(motor_velocity)>0):
            if(motor_velocity>0):# si no se apreta ninguna tecla, se frena solo
                motor_velocity -= 0.5
            elif (motor_velocity<0):
                motor_velocity += 0.5
    if (np.abs(motor_velocity)<dVel*0.1):
        brake_force=100
    else:
        brake_force=0
    #--set maximum steer angle
    if (steer_angle > max_steer_angle):
        steer_angle = max_steer_angle
    if (steer_angle < -max_steer_angle):
        steer_angle = -max_steer_angle
    sim.simxSetJointTargetPosition(clientID, steer_handle, steer_angle, sim.simx_opmode_oneshot )
    #--brake and motor can not be applied at the same time
    if(brake_force>0):
        sim.simxSetJointMaxForce(clientID, motor_handle, 0, sim.simx_opmode_oneshot)
    else:
        sim.simxSetJointMaxForce(clientID, motor_handle, motor_torque, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, motor_handle, motor_velocity, sim.simx_opmode_oneshot)

    sim.simxSetJointMaxForce(clientID, fr_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, fl_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, br_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, bl_brake_handle, brake_force, sim.simx_opmode_oneshot)
            
    #sim.simxSetFloatSignal(clientID, 'gyroY', gyro_angle, sim.simx_opmode_oneshot) # obtener el angulo del auto.
    #errorCode, gyroData = sim.simxGetStringSignal(clientID, 'gyro data', sim.simx_opmode_buffer)
    #posicion actual de manta:
    returnCode, pos = sim.simxGetObjectPosition(clientID, gps_handler, -1, sim.simx_opmode_blocking)
    # Obtenga datos v??lidos
    errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
    #Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
    ranges = sim.simxUnpackFloats(ranges)

    #gyroData = sim.simxUnpackFloats(gyroData)

    xf,yf,_ = pos

    if (xf != 0):
        delta_angle = math.atan2(yf,xf)
        angle = delta_angle
        #angle = delta_angle*180/math.pi

    #print(xf, angle)
  

    
    lidarData = clear_sensor(ranges)  
    #x = range(len(y))
    x = []
    y = []
    x, y = sense_obstacles(pos, angle, lidarData)
    

    # Dibuja el resultado
    line1.set_xdata(x)
    line1.set_ydata(y)
    figure.canvas.draw()
    figure.canvas.flush_events()
