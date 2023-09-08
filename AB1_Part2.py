import math 
import numpy as np
from roboticstoolbox import ET2, ET, ERobot, DHRobot, RevoluteDH, PrismaticDH, models
import matplotlib.pyplot as plt
from spatialmath.base import *

PLOT = True
PI = np.pi

def Q1(L1 = 1, L2 = 1, x= 0.5, y = 0.5):
    robot_2 = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)
    B = math.acos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    A = math.atan2(y, x) - math.atan2(L2 * math.sin(B), L1 + L2* math.cos(B))
    print(A,B)
    print(f"Fkine =\n{robot_2.fkine(q =[A,B])}")
    robot_2.teach(q = [A,B])

    print("Letra A")
    print(f"Fkine =\n{robot_2.fkine(q =[-0.4240,2.4188])}")
    robot_2.teach(q =[-0.4240,2.4188])
    print(robot_2.fkine(q =[-0.4240,2.4188]).printline())

    print(f"Fkine =\n{robot_2.fkine(q =[1.9948,-2.4188])}")
    robot_2.teach(q =[1.9948,-2.4188])
    print(robot_2.fkine(q =[1.9948,-24188]).printline())
   
    print("Letra B")
    robot_2 = robot_2* ET2.tx(qlim=[0,1])
    robot_2.teach(q = [A,B,1])


    print("Letra C")
    print(robot_2.fkine(q = [0,0.5,0.5]).printline())
    robot_2.teach(q = [0,0.5,0.5])

def Q2(L1 = 1, L2 = 1, L3 = 1, L4 = 1):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim([-1, 2])
    ax.set_ylim([-1, 2])
    ax.set_zlim([0, 2])

    O = transl(0, 0, 0)
    J1 = transl(0, 0, L1+L2) @ trotx(0)
    J2 = transl(0, 0, L1+ L2) @ trotx(PI/2)
    J3 = transl(L3,0, L1+ L2) @ trotx(PI/2)
    A = transl(L3 +L4,0, L1+ L2) @ trotx(PI/2)


    trplot(O, frame="O", color="k")
    trplot(J1, frame="1", color="b")
    trplot(J2, frame="2", color="r")
    trplot(J3, frame="3")
    trplot(A, frame="A",color="k")

    plt.show()

    e2 = RevoluteDH(d = L1+L2,alpha = PI/2,name = '1')
    e3 = RevoluteDH(a = L3 )
    e4 = RevoluteDH(a = L4)

    rob = DHRobot([e2,e3,e4], name = 'RRR')
    print(rob)

    T = rob.fkine_all(q=[0,0,0])
    print(T)
    rob.teach(q = [0,0,0])

def Q3(L0,L1,L2,D1,D3,D4):
    '''fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim([0, 3])
    ax.set_ylim([-1, 3])
    ax.set_zlim([-1, 3])

    O = transl(0, 0, 0)
    J1 = transl(0, 0, L0) @ trotx(0)
    J2 = transl(L1, 0, L0+D1) @ trotx(0)
    J3 = transl(L1+L2,0, L0+D1) @ trotx(PI)
    J4 = transl(L1+L2,0, L0+D1 - D3) @ trotx(PI)
    A = transl(L1+L2,0, L0 + D1 - D3 - D4) @ trotx(PI)

    trplot(O, frame="O", color="k")
    trplot(J1, frame="1", color="b")
    trplot(J2, frame="2", color="r")
    trplot(J3, frame="3")
    trplot(J4, frame="4", color="g")
    trplot(A, frame="A",color="k")

    plt.show()'''

    e1 = RevoluteDH(a = L1,d = D1)
    e2 = RevoluteDH(a = L2,alpha = PI)
    e3 = PrismaticDH(qlim = [0, D3])
    e4=  RevoluteDH(d = D4)
    rob = DHRobot([e1,e2,e3,e4], name = 'RRPR')
    print(rob)

    T = rob.fkine(q=[0,0,0.5,0])
    print(T)
    rob.teach(q = [0,0,0.5,0])

