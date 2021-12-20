# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 19:07:48 2021

@author: juan_
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# camino cuadrado
L = 10 # largo de cada lado, en puntos
D = 50 # distancia recorrida por cada lado [m]

x = []
y = []
L = L-1 # para poder ingresar el valor de puntos exacto
for i in range(L+1):
    x.append(D*i/L)
    y.append(0)
for i in range(L+1):
    y.append(D*i/L)
    x.append(D)
for i in range(L,-1,-1):
    x.append(D*i/L)
    y.append(D)
for i in range(L,-1,-1):
    y.append(D*i/L)
    x.append(0)
    
x = np.array(x) # arreglo de puntos de la coordenada x
y = np.array(y) # arreglo de puntos de la coordenada y

plt.plot(x, y, '.', color='blue',label="recorrido")
plt.xlabel("Coordenada X")
plt.ylabel("Coordenada Y")
plt.legend()
plt.show()
## 1.2 perfiles de velocidad

R = 0.225 # radio del robot [m]
# perfil de velocidades de circunferencia
#defición de los angulos en rad (se mantendran constante)
th_1 = (1/6)*np.pi #30 grados  # theta_1 = theta_r
th_2 = (2/3)*np.pi  # 120 grados
th_3 = (4/3)*np.pi  # 240 grados 

# el robot va a manter su angulo en todo momento, por lo que theta siempre se mantendra en cero

dT = 10 #delta tiempo [s]

A = (2/3)* np.array([[-np.sin(th_1), -np.sin(th_1+th_2), -np.sin(th_1+th_3)],
                     [np.cos(th_1), np.cos(th_1+th_2), np.cos(th_1+th_3)],
                     [0.5/R, 0.5/R, 0.5/R]]) # matriz de transformación 
A_inv = np.linalg.inv(A)

V = [] # matriz de velocidades

for i in range(x.size-1):
    V.append(np.array( np.matmul( (A_inv/dT) , np.transpose(np.array([x[i+1], y[i+1], 0]) - np.array([ x[i],y[i], 0])) )))    
V.append(np.array([0,0,0]))

V = np.array(V)

# separamos por vectores de velocidad [v1,v2,v3]
V1 = np.array(V[:,0])
V2 = np.array(V[:,1])
V3 = np.array(V[:,2])

## 1.3 

#Circular
#usaremos la misma matriz A calculada antes, y el mismo delta T

X0 = np.array([0,0,(1/6)*np.pi]) # vector de coordenadas en k # seteamos la primera posicion, en el origen
X1 = [] # vector de coordenadas en k + 1

X1.append(X0) # seteamos la primera posicion
for i in range(x.size-1):
    X1.append(X0 + dT*( np.matmul(A,np.transpose(np.array([V1[i],V2[i],V3[i]]))) ))
    X0 = X1[-1]

X1 = np.array(X1)

x_new = []
y_new = []
th = []

x_new = np.array(X1[:,0])
y_new = np.array(X1[:,1])
th = np.array(X1[:,2])


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