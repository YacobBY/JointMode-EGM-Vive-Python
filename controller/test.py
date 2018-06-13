import numpy as np
from math3D import *
R=np.random.rand(3,3)
t,u=angle_axis(R)
R=rotation_matrix(t,u)
print(t,u)
t,u=angle_axis(R)
print(t,u)
