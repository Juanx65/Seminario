# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 03:48:28 2021

@author: juan_
"""

import sim
import math
from cmath import sqrt
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
def sense_obstacles(pos, carAngle, lidarData, lidarRango):
    xdata=[]
    ydata=[]
    i = 0
    x1, y1,_ = pos
    for angle in np.linspace(0,(4/3)*math.pi,len(lidarData),False):
        if (lidarData[i] > 0 and lidarData[i] < lidarRango):
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
    if(xf-xi != 0 ):
        m = (yf-yi)/(xf-xi)
    else:
        m = 999999999999999
    c = yf - xf*m

    xdata = []
    ydata = []
    thdata = []
    
    x0 = xi # x_{k}
    y0 = yi # y_{k}
    x = x0
    y = y0
    while(1):
        if(m*m != -1):
            x = (math.sqrt(-c**2+c*(2*y-2*m*x0)+H**2*(m**2+1)-(y0-m*x0)**2) - c*m+m*y0+x0)/(m*m+1)  # x_{k+1}
        
        y = x*m+c

        th = math.atan2((y-y0),(x-x0))
        #print(th)

        xdata.append(x)
        ydata.append(y)
        thdata.append(th)

        x0 = x
        y0 = y

        if( (x >= xf-tolerancia/2 and x <= xf+tolerancia/2) and (y >= yf-tolerancia/2 and y <= yf+tolerancia/2) ):
            break
    return xdata, ydata, thdata
        
def calculate_vs (x0,y0,th0,x,y,th,dT):
    d = 0.6 # distancia entre las ruedas [m]

    alpha = 0
    V = 0

    Vcos = (x-x0)/(dT*math.cos(th0))
    Vsin = d*(th-th0)/dT
    
    alpha = math.atan2(Vsin,Vcos)
    if(math.cos(alpha) != 0):
        V = Vcos/math.cos(alpha)
    return V, alpha

def init_motor(brake_force,motor_velocity):
    sim.simxSetJointTargetVelocity(clientID, motor_handle, motor_velocity, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, fr_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, fl_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, br_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, bl_brake_handle, brake_force, sim.simx_opmode_oneshot)

def rodear_obstaculo(pos, alpha, H):
    x, y, _ = pos
    if(alpha != 0):
        c = y-math.tan(math.pi/2+alpha)*x
    else:
        c = y-999999999*x

    if(alpha != 0):
        sin = math.sin(alpha)
        cos = math.cos(alpha)
        cot = 1/math.tan(alpha)
        if(1/math.sin(alpha) != 0):
            x_new = sin*(sin*sqrt(-c**2+H**2*cot**2+H**2)+c*cos)
            x_new = x_new.real
            #print(x_new)
    else:
        x_new = x
    y_new = sqrt(H**2 - x_new**2)
    y_new = y_new.real
    return x_new, y_new

#establecemos la conexion con coppeliasim
clientID = connect(19999)
#obtener el handler para el car-like
_, car_like = sim.simxGetObjectHandle(clientID, 'Manta', sim.simx_opmode_blocking)

_, steer_handle = sim.simxGetObjectHandle(clientID,'steer_joint', sim.simx_opmode_blocking)
_, motor_handle = sim.simxGetObjectHandle(clientID,'motor_joint', sim.simx_opmode_blocking)
_, fl_brake_handle = sim.simxGetObjectHandle(clientID,'fl_brake_joint', sim.simx_opmode_blocking)
_, fr_brake_handle = sim.simxGetObjectHandle(clientID,'fr_brake_joint', sim.simx_opmode_blocking)
_, bl_brake_handle = sim.simxGetObjectHandle(clientID,'bl_brake_joint', sim.simx_opmode_blocking)
_, br_brake_handle = sim.simxGetObjectHandle(clientID,'br_brake_joint', sim.simx_opmode_blocking)

max_steer_angle=0.5235987
motor_torque=60
dVel=1
dSteer=0.1
steer_angle=0
motor_velocity=0#dVel*10
brake_force=0

#para el algoritmo de bug 1 (tangente)
_, goal_handler = sim.simxGetObjectHandle(clientID,'ReferenceFrame', sim.simx_opmode_blocking) #objeto que representa la meta ( goal)
_, posGoal = sim.simxGetObjectPosition(clientID, goal_handler, -1, sim.simx_opmode_blocking) # extraemos la posicion de la meta ( goal)

xgoal  = []
ygoal  = []
#inicializaciones de handlers y otros datos a extraer de coppelia
_, manta_handler = sim.simxGetObjectHandle(clientID,'Manta', sim.simx_opmode_blocking) #car like manta
_, lidar_handler = sim.simxGetObjectHandle(clientID,'fastHokuyo', sim.simx_opmode_blocking) #gps
_, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_streaming)# sensor
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
# Obtenga datos válidos
time.sleep(0.1)
_, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
#Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
ranges = sim.simxUnpackFloats(ranges)

#inicializacion animacion usando scatter
plt.ion()
figure = plt.figure(figsize=(10,10))
plt.ylim(-10,10)
plt.xlim(-10,10)

angle = 0 #inicializacion, angulo que indica direccion del carlike respcto al eje x
dT = 0.15
lidarRango = 3 # rango de deteccion de lidar

x = []
y = []
once = True
again = False

sim.simxSetJointMaxForce(clientID, motor_handle, motor_torque, sim.simx_opmode_oneshot)
    
while(1):  # making a loop

    #posicion actual de manta:
    _, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
    _, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)
    _, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
    ranges = sim.simxUnpackFloats(ranges)

    #obtenemos el angulo de la puta del auto respecto al centro del cuerpo ( respecto al eje x)
    xf,yf,_ = pos
    xi,yi,_ = bodyPos
    xg, yg,_ = posGoal
    
    angle = math.atan2(yf-yi,xf-xi)

    lidarData = clear_sensor(ranges,lidarRango)
    x, y = sense_obstacles(pos, angle, lidarData, lidarRango)

    if(once):
        dT = 0.15
        init_motor(0,0)
        xgoal, ygoal, thgoal = rect_generator(pos, posGoal, 0.7)
        once = False
        lenLine =  len(xgoal)
        i = 0
    
    for j in range(len(x)-1):
        if(i < lenLine):
            if(x[j] >= xgoal[i] - 0.5 and x[j] <= xgoal[i] + 0.5) and (y[j] >= ygoal[i]- 0.5 and y[j] <= ygoal[i]+ 0.5):
                x_rodear, y_rodear = rodear_obstaculo(pos, angle, 4)
                xgoal, ygoal, thgoal = rect_generator(pos, [x_rodear,y_rodear,0], 4)
                again = True
                lenLine =  len(xgoal)
                i=0
                dT = 0.5
                break
    
    if(i < lenLine):
        motor_velocity, steer_angle = calculate_vs(xf,yf,angle,xgoal[i],ygoal[i],thgoal[i],dT)

    if(i >= lenLine):
        if(again):
            once = True
            again = False
        init_motor(100,0)
        i = lenLine
    #--aplicamos velocidad al motor
    sim.simxSetJointTargetPosition(clientID, steer_handle, steer_angle, sim.simx_opmode_oneshot )
    sim.simxSetJointTargetVelocity(clientID, motor_handle, motor_velocity, sim.simx_opmode_oneshot)  
      
    time.sleep(dT)
    i += 1

    # Dibuja el resultado
    plt.scatter(xgoal, ygoal, c="blue")
    plt.scatter(xg, yg, c="red", marker="X") # punto de la meta
    plt.scatter(xf,yf,c="red") # posicion del sensor lidar en el mapa
    plt.scatter(x,y,c="black") # posicion de los obstaculos detectados por lidar
    figure.canvas.draw()
    figure.canvas.flush_events()