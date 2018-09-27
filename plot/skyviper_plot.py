#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv

time_lp = []
x_lp = []
y_lp = []
z_lp = []

with open('local_position.txt','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)
    for column in lines:
    	time_lp.append(float(column[0]))
    	x_lp.append(float(column[4]))
    	y_lp.append(float(column[5]))
    	z_lp.append(float(column[6]))

plt.plot(z_lp);
plt.show()

