# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 19:07:48 2021

@author: juan_
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# Camino circular
R1 = 500 # radio de la circunferencia 
pts = 100 # puntos necesarios para una semi-circunferencia.
x1 = []
y1 = []
for i in range(pts + 1):
    x1.append(2*R1*i/pts)
for i in range(pts+1):
    y1.append(np.sqrt(R1**2-(x1[i]-R1)**2))
for i in range(pts,-1,-1):
    y1.append(-np.sqrt(R1**2-(x1[i]-R1)**2))

x1 = x1 + x1[::-1]
x1 = np.array(x1) # arreglo de puntos de la coordenada x
y1 = np.array(y1) # arreglo de puntos de la coordenada y

plt.plot(x1, y1, '.', color='blue')
plt.show()

# Camino cuadrado

L = 100 # largo de cada lado, en puntos
D = 50 # distancia recorrida por cada lado 

x2 = []
y2 = []
L = L-1 # para poder ingresar el valor de puntos exacto
for i in range(L+1):
    x2.append(D*i/L)
    y2.append(0)
for i in range(L+1):
    y2.append(D*i/L)
    x2.append(D)
for i in range(L,-1,-1):
    x2.append(D*i/L)
    y2.append(D)
for i in range(L,-1,-1):
    y2.append(D*i/L)
    x2.append(0)
    
x2 = np.array(x2) # arreglo de puntos de la coordenada x
y2 = np.array(y2) # arreglo de puntos de la coordenada y

plt.plot(x2, y2, '.', color='blue')
plt.show()

# camino 8-invertido

R1 = 500 # radio de la circunferencia para cada lobulo
pts = 100 # puntos necesarios para una semi-circunferencia.
x3 = []
y3 = []
for i in range(2*pts + 2):
    x3.append(2*R1*i/pts)
for i in range(pts+1):
    y3.append(np.sqrt(R1**2-(x3[i]-R1)**2))
for i in range(pts+1):
    y3.append(-np.sqrt(R1**2-(x3[i]-R1)**2))
for i in range(pts,-1,-1):
    y3.append(np.sqrt(R1**2-(x3[i]-R1)**2))
for i in range(pts,-1,-1):
    y3.append(-np.sqrt(R1**2-(x3[i]-R1)**2))

x3 = x3 + x3[::-1]
x3 = np.array(x3) # arreglo de puntos de la coordenada x
y3 = np.array(y3) # arreglo de puntos de la coordenada y

plt.plot(x3, y3, '.', color='blue')
plt.show()


## 1.2 perfiles de velocidad

R = 22.5 # radio del robot
# perfil de velocidades de circunferencia
#defición de los angulos en rad (se mantendran constante)
th_1 = (1/6)*np.pi #30 grados  # theta_1 = theta_r
th_2 = (5/6)*np.pi #150 grados
th_3 = (3/2)*np.pi #270 grados

# el robot va a manter su angulo en todo momento, por lo que theta siempre se mantendra en cero

dT = 10 #delta tiempo, un misterio

A = (2/3)* np.array([[-np.sin(th_1), -np.sin(th_1+th_2), -np.sin(th_1+th_3)],
                     [np.cos(th_1), np.cos(th_1+th_3), np.cos(th_1+th_3)],
                     [0.5*R, 0.5*R, 0.5*R]]) # matriz de transformación 
A_inv = np.linalg.inv(A)

V1 = [] # matriz de velocidades

for i in range(x1.size-1):
    V1.append(np.array( np.matmul( (A_inv/dT) , np.transpose(np.array([x1[i+1], y1[i+1], 0]) - np.array([ x1[i],y1[i], 0])) )))    
V1.append(np.array([0,0,0]))

V1 = np.array(V1)

# separamos por vectores de velocidad [v1,v2,v3]
V1_1 = np.array(V1[:,0])
V1_2 = np.array(V1[:,1])
V1_3 = np.array(V1[:,2])

## 1.3 re hacer el circulo

#Circular
#usaremos la misma matriz A calculada antes, y el mismo delta T

X1_0 = np.array([0,0,0]) # vector de coordenadas en k # seteamos la primera posicion, en el origen
X1_1 = [] # vector de coordenadas en k + 1

X1_1.append(X1_0) # seteamos la primera posicion
for i in range(x1.size):
    X1_1.append(X1_0 + dT*( np.matmul(A,np.transpose(np.array([V1_1[i],V1_2[i],V1_3[i]]))) ))
    X1_0 = X1_1[-1]

X1_1 = np.array(X1_1)

x = np.array(X1_1[:,0])
y = np.array(X1_1[:,1])
th = np.array(X1_1[:,2])


## animación

x_data = []
y_data = []

fig, ax = plt.subplots()
ax.set_xlim(0,2*R1)
ax.set_ylim(-R1,R1)
line, = ax.plot(0,0)

def animation_frame(i):
    x_data.append(x[i])
    y_data.append(y[i])
    
    line.set_xdata(x_data)
    line.set_ydata(y_data)
    return line,

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(x1.size),interval=10)
plt.show()



