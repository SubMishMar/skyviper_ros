#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv

time_lp = []
x_lp = []
y_lp = []
z_lp = []

with open('global_pose.txt','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)
    for column in lines:
    	time_lp.append(float(column[0]))
    	x_lp.append(float(column[4]))
    	y_lp.append(float(column[5]))
    	z_lp.append(float(column[6]))

time_mp = []
x_mp = []
y_mp = []
z_mp = []

with open('global_pose_kf.txt','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)
    for column in lines:
    	time_mp.append(float(column[0]))
    	x_mp.append(float(column[4]))
    	y_mp.append(float(column[5]))
    	z_mp.append(float(column[6]))

fig, axs = plt.subplots(3, 1)
axs[0].plot(time_lp, x_lp);
axs[0].hold(True)
axs[0].plot(time_mp, x_mp);
axs[0].hold(False)
axs[0].legend(['Without KF', 'With KF'])
axs[0].set_ylabel('X')
axs[0].grid()

axs[1].plot(time_lp, y_lp);
axs[1].hold(True)
axs[1].plot(time_mp, y_mp);
axs[1].hold(False)
axs[1].legend(['Without KF', 'With KF'])
axs[1].set_ylabel('Y')
axs[1].grid()

axs[2].plot(time_lp, z_lp);
axs[2].hold(True)
axs[2].plot(time_mp, z_mp);
axs[2].hold(False)
axs[2].legend(['Without KF', 'With KF'])
axs[2].set_xlabel('time')
axs[2].set_ylabel('Z')
axs[2].grid()

plt.show()


