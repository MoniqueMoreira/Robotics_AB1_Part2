import math 
import numpy as np
from roboticstoolbox import ET2, ET, ERobot, DHRobot, RevoluteDH, PrismaticDH, models
import matplotlib.pyplot as plt
from spatialmath.base import *

PLOT = True
PI = np.pi
e1 = RevoluteDH(alpha = 0)
e3 = RevoluteDH(a = 1)
e4 = RevoluteDH(a = 1)

rob = DHRobot([e1,e3,e4], name = 'RRR')
print(rob)

T = rob.fkine_all(q=[0,0.5,0])
print(T)
print()
print("0T1", T[1]@T[0])
print()

print("1T2", T[2]@T[1])
print()
#print("2T3", np.dot(T[2],T[3]))
#print("3T0", np.dot(T[3],T[0]))
rob.teach(q = [0,0,0])