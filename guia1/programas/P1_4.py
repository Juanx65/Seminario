# -*- coding: utf-8 -*-
"""
Created on Sat Nov  6 16:27:15 2021

@author: juan_
"""

## P1.4 

## implementación de movimiento como uniciclo

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# camino cuadrado
L = 100 # largo de cada lado, en puntos
D = 50 # distancia recorrida por cada lado [m]
G = 10 # tiempo de giro [en puntos]

x = []
y = []
th = []

th.append(0) # angulo theta_r inicial
L = L-1 # para poder ingresar el valor de puntos exacto
for i in range(L+1):
    x.append(D*i/L)
    y.append(0)
    th.append(th[-1])
for i in range(G+1):
    th.append((np.pi/2)*(i/G)) # llega a 90 grados
    x.append(x[-1])
    y.append(y[-1])
for i in range(L+1):
    y.append(D*i/L)
    x.append(D)
    th.append(th[-1])
for i in range(G+1): 
    th.append((np.pi/2)+ (np.pi/2)*(i/G)) # llega a 180 grados
    x.append(x[-1])
    y.append(y[-1])
for i in range(L,-1,-1):
    x.append(D*i/L)
    y.append(D)
    th.append(th[-1])
for i in range(G+1): 
    th.append((np.pi)+(np.pi/2)*(i/G)) # llega a -90 grados
    x.append(x[-1])
    y.append(y[-1])
for i in range(L,-1,-1):
    y.append(D*i/L)
    x.append(0)
    th.append(th[-1])
for i in range(G+1): 
    th.append((np.pi+np.pi/2)+(np.pi/2)*(i/G)) # llega a 0 grados
    x.append(x[-1])
    y.append(y[-1])
    
x = np.array(x) # arreglo de puntos de la coordenada x
y = np.array(y) # arreglo de puntos de la coordenada y
th = np.array(th)# arreglo de puntos el angulo theta_r

plt.plot(x, y, '.', color='blue')
plt.show()

## 1.4 perfiles de velocidad

R = 0.225 # radio del robot [m]
# perfil de velocidades para el recorrido cuadrado
#defición de los angulos en rad (se mantendran constante)
th_2 = (2/3)*np.pi  # 120 grados
th_3 = (4/3)*np.pi  # 240 grados 

# el robot va a manter su angulo en todo momento, por lo que theta siempre se mantendra en cero

dT = 10 #delta tiempo, en segundos
A = []
for i in range(x.size):
    A.append( (2/3)* np.array([[-np.sin(th[i]), -np.sin(th[i]+th_2), -np.sin(th[i]+th_3)],
                     [np.cos(th[i]), np.cos(th[i]+th_2), np.cos(th[i]+th_3)],
                     [0.5/R, 0.5/R, 0.5/R]]) ) # matriz de transformación 
A = np.array(A)

V = [] # matriz de velocidades

for i in range(x.size-1):
    V.append(np.array( np.matmul( (np.linalg.inv(A[i])/dT) , np.transpose(np.array([x[i+1], y[i+1], th[i+1]]) - np.array([ x[i],y[i],th[i]])) )))    
V.append(np.array([0,0,0]))

V = np.array(V)

# separamos por vectores de velocidad [v1,v2,v3]
V1 = np.array(V[:,0])
V2 = np.array(V[:,1])
V3 = np.array(V[:,2])

## 1.3 
#usaremos la misma matriz A calculada antes, y el mismo delta T

X0 = np.array([0,0,0]) # vector de coordenadas en k # seteamos la primera posicion, en el origen
X1 = [] # vector de coordenadas en k + 1

X1.append(X0) # seteamos la primera posicion
for i in range(x.size-1):
    X1.append(X0 + dT*( np.matmul(A[i],np.transpose(np.array([V1[i],V2[i],V3[i]]))) ))
    X0 = X1[-1]

X1 = np.array(X1)

x_new = []
y_new = []
th_new = []

x_new = np.array(X1[:,0])
y_new = np.array(X1[:,1])
th_new = np.array(X1[:,2])


## animación

x_data = []
y_data = []

fig, ax = plt.subplots()
ax.set_xlim(-10,D+10)
ax.set_ylim(-10,D+10)
line, = ax.plot(0,0)

def animation_frame(i):
    x_data.append(x_new[i])
    y_data.append(y_new[i])
    
    line.set_xdata(x_data)
    line.set_ydata(y_data)
    return line,

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(x.size),interval=dT)
plt.show()

plt.plot(x_new, y_new, '.', color='blue')
plt.show()


#########################################
#########################################
## implementación uniciclo para comprobar


## 1.4 perfiles de velocidad para uniciclo

# el robot va a manter su angulo en todo momento, por lo que theta siempre se mantendra en cero

dT = 10 #delta tiempo, un misterio
Au = []
for i in range(x.size):
    Au.append(np.array([[np.cos(th[i]), 0],
                     [np.sin(th[i]), 0],
                     [0, 1]] )) # matriz de transformación para el unicilo
Au = np.array(Au)

Vu = [] # matriz de velocidades [v,w] uniciclo

for i in range(x.size-1):
    Vu.append(np.array( np.matmul( (np.linalg.pinv(Au[i])/dT) , np.transpose(np.array([x[i+1], y[i+1], th[i+1]]) - np.array([ x[i],y[i],th[i]])) )))    
Vu.append(np.array([0,0]))

Vu = np.array(Vu)

# separamos por vectores de velocidad [v,w]
v_u = np.array(Vu[:,0])
w_u = np.array(Vu[:,1])

## 1.3 
#usaremos la misma matriz A calculada antes, y el mismo delta T

Xu0 = np.array([0,0,0]) # vector de coordenadas en k # seteamos la primera posicion, en el origen
Xu1 = [] # vector de coordenadas en k + 1

Xu1.append(Xu0) # seteamos la primera posicion
for i in range(x.size-1):
    Xu1.append(Xu0 + dT*( np.matmul(Au[i],np.transpose(np.array([v_u[i],w_u[i]])))))
    Xu0 = Xu1[-1]

Xu1 = np.array(Xu1)

xu_new = []
yu_new = []
thu_new = []

xu_new = np.array(Xu1[:,0])
yu_new = np.array(Xu1[:,1])
thu_new = np.array(Xu1[:,2])

## animación

x_data = []
y_data = []

fig, ax = plt.subplots()
ax.set_xlim(-10,D+10)
ax.set_ylim(-10,D+10)
line, = ax.plot(0,0)

def animation_frame(i):
    x_data.append(xu_new[i])
    y_data.append(yu_new[i])
    
    line.set_xdata(x_data)
    line.set_ydata(y_data)
    return line,

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(x.size),interval=dT)
plt.show()

plt.plot(xu_new, yu_new, '.', color='blue')
plt.show()

v_time = [] # tiempo para graficar las velocidades
## grafico de la señal de control vs tiempo
for i in range(x.size):
    v_time.append(dT*i)
v_time = np.array(v_time)

plt.plot(v_time,V1, '.', color='blue',label="v1")
plt.plot(v_time,V2, '.', color='green',label="v2")
plt.plot(v_time,V3, '.', color='red',label="v3")
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidad [m/s]")
plt.legend()
plt.show()
