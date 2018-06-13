import ABB120
import numpy as np

J=np.array([32.28,5.84,27.35,-46.82,64.22,33.54])
H_tool,xyz,R,Q=ABB120.FK(J)
print(xyz)
print(Q)
print(H_tool)

print("**********************")
sol=ABB120.IK(H_tool)
print(sol[0])
