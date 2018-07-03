import ABB120
import numpy as np

J=np.array([0,0,30,0,60,0])
H_tool,xyz,R,Q=ABB120.FK(J)
print(xyz)
print(Q)
print(H_tool)

print("**********************")
sol=ABB120.IK(H_tool)
print(sol)
print(sol[0])
