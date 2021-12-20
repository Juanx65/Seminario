# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 19:12:22 2021

@author: juan_
"""

import sim
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
 
# Desconectar conexión anterior
sim.simxFinish(-1)
 #conectarse al servidor
clientID = sim.simxStart ('127.0.0.1', 19997, True, True, 5000, 5)
 
if clientID != -1:
    print('Server is connected!')
else:
    print('Server is unreachable!') 
    sys.exit(0)
 
 # La primera vez para obtener datos, los datos no son válidos
returnCode, gps_handler = sim.simxGetObjectHandle(clientID,'GPS', sim.simx_opmode_blocking)

errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_streaming)
time.sleep(0.1)

 
while(1):
    time.sleep(0.1)
    returnCode, pos = sim.simxGetObjectPosition(clientID, gps_handler, -1, sim.simx_opmode_blocking)
    print(pos)
    # Obtenga datos válidos
    errorCode, ranges = sim.simxGetStringSignal(clientID, 'scan ranges', sim.simx_opmode_buffer)
    #Convertir cadena a lista flotante, el valor en la lista es el valor medido del radar
    ranges = sim.simxUnpackFloats(ranges)
    x = range(len(ranges))
    y = ranges
    # Dibuja el resultado
    plt.scatter(x, y)
    plt.show()