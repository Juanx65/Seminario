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

 # dist = distancia maxima para detectar objetos
def clear_sensor(array, dist):
    for i in range(len(array)):
        if array[i] > dist:
            array[i] = 0
    return array

#retorna las posiciones donde existen obstaluclos detectados por lidar
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
# genera los puntod necesarios para trazar una recta entre pos_in y pos_goal con una separeacion de H
def rect_generator(pos_in, pos_goal, H):
    tolerancia = H
    xi, yi,_ = pos_in
    xf, yf,_ = pos_goal
    m = (yf-yi)/(xf-xi)
    c = yf - xf*m
    
    xdata = []
    ydata = []
    
    x0 = xi # x_{k}
    y0 = yi # y_{k}
    x = x0
    y = y0
    xdata.append(x)
    ydata.append(y)
    i = 0
    while(1):
        i += 1
        if(m*m != -1):
            x = (math.sqrt(-c**2+c*(2*y-2*m*x0)+H**2*(m**2+1)-(y0-m*x0)**2) - c*m+m*y0+x0)/(m*m+1)  # x_{k+1}
        
        y = x*m+c

        xdata.append(x)
        ydata.append(y)

        x0 = x
        y0 = y

        print(x, y)

        if( (x >= xf-tolerancia/2 and x <= xf+tolerancia/2) and (y >= yf-tolerancia/2 and y <= yf+tolerancia/2) ):
            break
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

#para el algoritmo de bug 1 (tangente)
returnCode, goal_handler = sim.simxGetObjectHandle(clientID,'ReferenceFrame', sim.simx_opmode_blocking) #objeto que representa la meta ( goal)
returnCode, posGoal = sim.simxGetObjectPosition(clientID, goal_handler, -1, sim.simx_opmode_blocking) # extraemos la posicion de la meta ( goal)

xgoal  = []
ygoal  = []
#inicializaciones de handlers y otros datos a extraer de coppelia
returnCode, manta_handler = sim.simxGetObjectHandle(clientID,'Manta', sim.simx_opmode_blocking) #car like manta
returnCode, lidar_handler = sim.simxGetObjectHandle(clientID,'fastHokuyo', sim.simx_opmode_blocking) #gps
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_streaming)# sensor
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
# Obtenga datos válidos
time.sleep(0.1)
errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
ranges = sim.simxUnpackFloats(ranges)

#inicializacion animacion usando scatter
plt.ion()
figure = plt.figure(figsize=(10,10))
plt.ylim(-10,10)
plt.xlim(-10,10)

angle = 0 #inicializacion, angulo que indica direccion del carlike respcto al eje x
once = True
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
    xg, yg,_ = posGoal
    if (xf != 0):
        angle = math.atan2(yf-yi,xf-xi)
    
    lidarData = clear_sensor(ranges,3)   # solo nos quedamos con un rango especifico de valores detectados por lidar 

    if(once):
        xgoal, ygoal = rect_generator(pos, posGoal, 0.5)
        once = False

    # x, y representan las posiciones de los obstaculos detectados por lidar
    x = []
    y = []
    x, y = sense_obstacles(pos, angle, lidarData)
    
    # Dibuja el resultado
    plt.scatter(xgoal, ygoal, c="blue")
    plt.scatter(xg, yg, c="purple") # punto de la meta
    plt.scatter(xf,yf,c="red") # posicion del sensor lidar en el mapa
    plt.scatter(x,y,c="black") # posicion de los obstaculos detectados por lidar
    figure.canvas.draw()
    figure.canvas.flush_events()