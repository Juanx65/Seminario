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

def clear_sensor(array, dist): # dist = distancia maxima para detectar objetos
    for i in range(len(array)):
        if array[i] > dist:
            array[i] = 0
    return array

def sense_obstacles(pos, carAngle, lidarData):
    xdata=[]
    ydata=[]
    i = 0
    x1, y1,_ = pos
    for angle in np.linspace(0,(4/3)*math.pi,len(lidarData),False):
        if (lidarData[i] > 1 and lidarData[i] < 3):
            x2, y2 = (x1 + lidarData[i]*math.cos(angle+(carAngle-(4/3)*math.pi/2)), y1 + lidarData[i]*math.sin(angle+(carAngle-(4/3)*math.pi/2)))
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


returnCode, manta_handler = sim.simxGetObjectHandle(clientID,'Manta', sim.simx_opmode_blocking) #car like manta
returnCode, lidar_handler = sim.simxGetObjectHandle(clientID,'fastHokuyo', sim.simx_opmode_blocking) #gps
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_streaming)# sensor
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
# Obtenga datos válidos
time.sleep(0.1)
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
ranges = sim.simxUnpackFloats(ranges)

plt.ion()
#figure, ax = plt.subplot(figsize=(10,10))
figure = plt.figure(figsize=(10,10))
plt.ylim(-10,10)
plt.xlim(-10,10)
#line1, = ax.plot(range(len(ranges)),  ranges )

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
            
    #posicion actual de manta:
    returnCode, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
    retunCode, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)
    # Obtenga datos válidos del lidar
    errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
    #Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
    ranges = sim.simxUnpackFloats(ranges)

    #obtenemos el angulo de la puta del auto respecto al centro del cuerpo ( respecto al eje x)
    xf,yf,_ = pos
    xi,yi,_ = bodyPos

    if (xf != 0):
        delta_angle = math.atan2(yf-yi,xf-xi)
        angle = delta_angle
    
    lidarData = clear_sensor(ranges,3)   # solo nos quedamos con un rango especifico de valores detectados por lidar 

    # x, y representan las posiciones de los obstaculos detectados por lidar
    x = []
    y = []
    x, y = sense_obstacles(pos, angle, lidarData)
    

    # Dibuja el resultado
    plt.scatter(xf,yf,c="red") # posicion del sensor lidar en el mapa
    plt.scatter(x,y,c="black") # posicion de los obstaculos detectados por lidar
    figure.canvas.draw()
    figure.canvas.flush_events()
