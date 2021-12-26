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
def sense_obstacles(pos, carAngle, lidarData):
    xdata=[]
    ydata=[]
    i = 0
    x1, y1,_ = pos
    for angle in np.linspace(0,(4/3)*math.pi,len(lidarData),False):
        if(lidarData[i] > 0 ):
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
    
    alpha = math.atan2((yf-yi),(xf-xi))

    xdata = []
    ydata = []
    thdata = []
    
    x0 = xi # x_{k}
    y0 = yi # y_{k}

    #print("entre")
    while(1):
        
        x = math.cos(alpha)*H + x0
        y = math.sin(alpha)*H + y0

        if((y-y0) != 0 and (x-x0) != 0):
            th = math.atan2((y-y0),(x-x0))
        else:
            th = 0

        xdata.append(x)
        ydata.append(y)
        thdata.append(th)

        x0 = x
        y0 = y
        if( (np.linalg.norm(np.array((x,y))-np.array((xf,yf))) < 2*H)):
            break
        elif( ( (x >= xf-tolerancia and x <= xf+tolerancia) and (y >= yf-tolerancia and y <= yf+tolerancia) ) ):
            break
    #print("sali")
    xdata.append(xf)
    ydata.append(yf)
    thdata.append(th)
    return xdata, ydata, thdata

# calcula la velocidad necesaria para llegar del punto x,y,Th al x0,y0,Th0
def calculate_vs (x0,y0,th0,x,y,th,dT):
    d = 0.6 # distancia entre las ruedas [m]

    alpha = 0
    V = 0

    if(math.cos(th0) != 0 ):
        Vcos = (x-x0)/(dT*math.cos(th0))
    else:
        Vcos = float("inf")
    Vsin = d*(th-th0)/dT
    
    alpha = math.atan2(Vsin,Vcos)
    if(math.cos(alpha) != 0):
        V = Vcos/math.cos(alpha)
    else:
        V = float("inf")
    return V, alpha

# permite frenar o dar velocidad al motor
def init_motor(brake_force,motor_velocity,steer_angle):
    sim.simxSetJointMaxForce(clientID, fr_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, fl_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, br_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointMaxForce(clientID, bl_brake_handle, brake_force, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, steer_handle, steer_angle, sim.simx_opmode_oneshot )
    sim.simxSetJointTargetVelocity(clientID, motor_handle, motor_velocity, sim.simx_opmode_oneshot)

def rodear_paralelo(pos, pos_obs, co):
    x, y,_ = pos
    xo, yo = pos_obs
    ca = np.linalg.norm(np.array((xo,yo))-np.array((x,y)))
    alpha = math.atan2(co,ca)
    H = math.sqrt(co**2+ca**2)
    xn = xo - H*math.cos(alpha)
    yn = yo - H*math.sin(alpha)
    return xn, yn
def rodear_paralelo_lado(pos, pos_obs, ca):
    x, y,_ = pos
    xo, yo = pos_obs
    H = np.linalg.norm(np.array((xo,yo))-np.array((x,y)))
    if(ca > H):
        ca = H-0.001
    co = math.sqrt(H**2-ca**2)
    alpha = math.atan2(co,ca)
    xn = xo + H*math.cos(alpha)
    yn = yo + H*math.sin(alpha)
    return xn, yn
# trata de rodear el obstaculo
def rodear_obstaculo(camino,lc ,dT, paso, lidarRango):

    #listas de posiciones
    xc, yc, thc = camino

    enCamino = False
    obstaculos = []
    x_rodear = []
    y_rodear = []
    while(~enCamino):
        _, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
        _, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)
        _, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
        ranges = sim.simxUnpackFloats(ranges)

        #pos actual
        x, y,_ = pos
        xi,yi,_ = bodyPos
        angle = math.atan2(y-yi,x-xi)

        lidarData = clear_sensor(ranges,lidarRango)
        xo, yo = sense_obstacles(pos, angle, lidarData)

        for i in range(len(xc[lc:])):
            if(x >= xc[i+lc] - paso and x <= xc[i+lc] + paso) and (y >= yc[i+lc] - paso and y <= yc[i+lc] + paso):
                enCamino = True
                return i
        
        for i in range(len(xo)):
            init_motor(100,0,0)
            _, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
            _, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)

            #pos actual
            x, y,_ = pos
            xi,yi,_ = bodyPos
            angle = math.atan2(y-yi,x-xi)
            angle_o = math.atan2(yo[i]-y,xo[i]-x)

            if(angle_o > angle - math.pi/8 and angle_o < angle + math.pi/8):
                motor_velocity = 10
                steer_angle = math.pi/2
                init_motor(0,motor_velocity,steer_angle)
                time.sleep(dT)
                break
            else:
                x_rodear, y_rodear = rodear_paralelo(pos, [xo[i],yo[i]], paso)
            
            xgoal, ygoal, thgoal = rect_generator(pos, [x_rodear,y_rodear,0], paso)

            for j in range(len(xgoal)):

                _, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
                _, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)

                #pos actual
                x, y,_ = pos
                xi,yi,_ = bodyPos
                angle = math.atan2(y-yi,x-xi)
                
                motor_velocity, steer_angle = calculate_vs(x,y,angle,xgoal[j],ygoal[j],thgoal[j],dT)

                #--aplicamos velocidad al motor
                init_motor(0,motor_velocity,steer_angle)
                
                time.sleep(dT)

                # Dibuja el resultado
                plt.scatter(xgoal, ygoal, c="cyan")
                plt.scatter(x,y,c="red") # posicion del sensor lidar en el mapa
                plt.scatter(xo,yo,c="black") # posicion de los obstaculos detectados por lidar
                figure.canvas.draw()
                figure.canvas.flush_events()  

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

sim.simxSetJointMaxForce(clientID, motor_handle, motor_torque, sim.simx_opmode_oneshot)

#Inicializacion de animacion usando scatter
plt.ion()
figure = plt.figure(figsize=(10,10))
plt.ylim(-10,10)
plt.xlim(-10,10)

# inicializacion de variables

angle = 0 # representa la direccion del carlike respecto al origen
dT = 0.15 # delta de tiempo para las velocidades
lidarRango = 3 # rango del sensor lidar a detectar
paso = 0.7  # paso de cada punto al generar los caminos
xgoal  = []
ygoal  = []
l = 0 # contador para seguir la linea

_, posGoal = sim.simxGetObjectPosition(clientID, goal_handler, -1, sim.simx_opmode_blocking) # extraemos la posicion de la meta ( goal)
_, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
_, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)
xgoal, ygoal, thgoal = rect_generator(pos, posGoal, paso)

while(l < len(xgoal)): #loop para mover carlike hasta q termine el algortmo 
    
    #posicion actual de manta:
    _, pos = sim.simxGetObjectPosition(clientID, lidar_handler, -1, sim.simx_opmode_blocking)
    _, bodyPos = sim.simxGetObjectPosition(clientID, manta_handler,-1, sim.simx_opmode_blocking)
    _, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
    ranges = sim.simxUnpackFloats(ranges)

    #inicializaciones
    x,y,_ = pos
    xb,yb,_ = bodyPos
    xg, yg,_ = posGoal

    angle = math.atan2(y-yb,x-xb)

    lidarData = clear_sensor(ranges,lidarRango)
    xo, yo = sense_obstacles(pos, angle, lidarData)
    
    k = round(lidarRango/paso)-2
    if(l+k < len(xgoal)):
        for i in range(len(xo)):
            if((xo[i] >= xgoal[l+k] - paso and xo[i] <= xgoal[l+k] + paso) and (yo[i] >= ygoal[l+k] - paso and yo[i] <= ygoal[l+k]+ paso)):
                init_motor(100,0,0)
                j = rodear_obstaculo([xgoal, ygoal, thgoal],l, dT, paso, lidarRango)
                l += j
                break 
    #Seguir linea a la meta:
    motor_velocity, steer_angle = calculate_vs(x,y,angle,xgoal[l],ygoal[l],thgoal[l],dT)
    #--aplicamos velocidad al motor
    init_motor(0,motor_velocity,steer_angle)
    time.sleep(dT)
    l += 1

    # Dibuja el resultado
    plt.scatter(xgoal, ygoal, c="blue")
    plt.scatter(xg, yg, c="red", marker="X") # punto de la meta
    plt.scatter(x,y,c="red") # posicion del sensor lidar en el mapa
    plt.scatter(xo,yo,c="black") # posicion de los obstaculos detectados por lidar
    figure.canvas.draw()
    figure.canvas.flush_events()   
