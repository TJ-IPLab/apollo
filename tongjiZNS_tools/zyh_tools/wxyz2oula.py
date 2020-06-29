#!usr/bin/env python
import math as m
def oula2wxyz(x,y,z):
	x1=((y*0.5)/(180))*m.pi
	y1=((x*0.5)/(180))*m.pi
	z1=((z*0.5)/(180))*m.pi
	w=m.cos(x1)*m.cos(y1)*m.cos(z1)+m.sin(x1)*m.sin(y1)*m.sin(z1)
	w_x=m.sin(x1)*m.cos(y1)*m.cos(z1)-m.cos(x1)*m.sin(y1)*m.sin(z1)
	w_y=m.cos(x1)*m.sin(y1)*m.cos(z1)+m.sin(x1)*m.cos(y1)*m.sin(z1)
	w_z=m.cos(x1)*m.cos(y1)*m.sin(z1)-m.sin(x1)*m.sin(y1)*m.cos(z1)
	return w,w_x,w_y,w_z
if __name__== '__main__':
	x=input('inputx=')
	y=input('inputy=')
	z=input('inputz=')
	x=float(x)
	y=float(y)
	z=float(z)
	print(oula2wxyz(x,y,z))
