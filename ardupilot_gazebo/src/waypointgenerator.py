#!/usr/bin/env python

import rospy
import numpy as np
import random
import matplotlib.pyplot as plt

if __name__=="__main__":
    disp = 0.5
    displacements = [-disp, 0, disp]
    setpoint = [2, 2]
    x = [setpoint[0]]
    y = [setpoint[1]]
    time = [0]
    for i in range(20):
        
        dX = random.choice(displacements)
        dY = random.choice(displacements)
        new_setpointX = setpoint[0] + dX
        new_setpointY = setpoint[1] + dY
        print(['Generated Setpoint:', new_setpointX, new_setpointY])
        if 0 <= new_setpointX <= 4 and 0 <= new_setpointY <= 4:
            setpoint[0] = new_setpointX
            setpoint[1] = new_setpointY
        else:
            print('Setpoint outta bounds')
        print(['Accepted Setpoint: ', setpoint])
        print('\n')
        x.append(setpoint[0])
        y.append(setpoint[1])
        time.append(i+1)
    
    plt.subplot(2, 1, 1)
    plt.plot(time, x, label='X setpoint')
    plt.plot(time, y, label='Y setpoint')

    plt.xlabel('time')
    plt.ylabel('x and y')

    plt.title("Plot of Setpoints")
    plt.grid()
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(x, y)
    plt.grid()
    plt.show()


        

                
