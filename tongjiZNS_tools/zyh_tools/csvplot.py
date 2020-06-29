#!/usr/bin/env python
# coding=utf-8
import csv
import matplotlib.pyplot as plt 
import numpy as np
from scipy import interpolate

path='gps.csv'
f=open(path,'r')
d=csv.reader(f)
x_values = []
y_values = []
for line in d:
	string=str(line)
	L=string.split(';')
	x_values.append(float(L[2]))
	y_values.append(float(L[3]))
f.close()
x = np.array(x_values)
y = np.array(y_values)
x_new = np.linspace(x.min(),x.max(),100000)

func=interpolate.interp1d(x,y,kind='cubic')
y_smooth = func(x_new)
plt.plot(x_new,y_smooth,c='red')
plt.plot(x_values,y_values)
plt.scatter(x_values,y_values,s=10)
plt.axis('equal')
plt.show()
