#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv

time_mp = []
x_mp = []
y_mp = []
z_mp = []

with open('mocap_position.txt','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)
    for column in lines:
    	time_mp.append(float(column[0]))
    	x_mp.append(float(column[4]))
    	y_mp.append(float(column[5]))
    	z_mp.append(float(column[6]))

fig, axs = plt.subplots(3, 1)
axs[0].plot(time_mp, x_mp)
axs[0].set_ylabel('X')
axs[0].grid()

axs[1].plot(time_mp, y_mp)
axs[1].set_ylabel('Y')
axs[1].grid()

axs[2].plot(time_mp, z_mp)
axs[2].set_ylabel('Z')
axs[2].grid()

plt.show()

