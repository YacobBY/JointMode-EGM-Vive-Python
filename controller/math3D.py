import numpy as np
from math import sqrt, asin, sin, cos

def angle_axis(R):
    u=np.zeros(3)
    u[0]=R[2,1]-R[1,2]
    u[1]=R[0,2]-R[2,0]
    u[2]=R[1,0]-R[0,1]
    mag=sqrt(u[0]**2+u[1]**2+u[2]**2)
    u=u/mag
    angle=asin(mag/2)

    return angle, u

def rotation_matrix(t, u):
    R=np.array([[cos(t)+u[0]**2*(1-cos(t)),         u[0]*u[1]*(1-cos(t))-u[2]*sin(t),   u[0]*u[2]*(1-cos(t))+u[1]*sin(t)],
               [u[0]*u[1]*(1-cos(t))+u[2]*sin(t),   cos(t)+u[1]**2*(1-cos(t)),          u[1]*u[2]*(1-cos(t))-u[0]*sin(t)],
               [u[0]*u[2]*(1-cos(t))-u[1]*sin(t),   u[1]*u[2]*(1-cos(t))+u[0]*sin(t),   cos(t)+u[2]**2*(1-cos(t))       ]])
    return R
