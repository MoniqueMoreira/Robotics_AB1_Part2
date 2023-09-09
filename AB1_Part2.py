import math as m
import numpy as np
from roboticstoolbox import ET2, ET, ERobot, DHRobot, RevoluteDH, PrismaticDH, models
import matplotlib.pyplot as plt
from spatialmath.base import *

PLOT = True
PI = np.pi

def Q1(L1 = 1, L2 = 1, x= 0.5, y = 0.5):
    
    Rob = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)
    B = m.acos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    A = m.atan2(y, x) - m.atan2(L2 * m.sin(B), L1 + L2* m.cos(B))
    print(A,B)

    print(f"Fkine =\n{Rob.fkine(q =[A,B])}")
    Rob.teach(q = [A,B])

    print("Letra A")
    print(f"Fkine =\n{Rob.fkine(q =[-0.4240,2.4188])}")
    Rob.teach(q =[-0.4240,2.4188])
    print(Rob.fkine(q =[-0.4240,2.4188]).printline())

    print(f"Fkine =\n{Rob.fkine(q =[1.9948,-2.4188])}")
    Rob.teach(q =[1.9948,-2.4188])
    print(Rob.fkine(q =[1.9948,-24188]).printline())
   
    print("Letra B")
    L1 = 2
    L2 = 1.5
    Rob = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)* ET2.tx(qlim=[0,2])
    Rob.teach(q = [A,B,1])
    Rob.teach(q = [0,B,2])


    q = [0,0.5,0.5]
    print(f"Fkine =\n{Rob.fkine(q)}")
    print(Rob.fkine(q).printline())
    Rob.teach(q)

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

def Q3(q = [0,0,0.5,0],L0 = 1,L1=1,L2=1,D1=0.2,D3=1,D4=0.2):
    fig = plt.figure()
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

    plt.show()

    e1 = RevoluteDH(a = L1,d = D1)
    e2 = RevoluteDH(a = L2,alpha = PI)
    e3 = PrismaticDH(qlim = [0, D3])
    e4=  RevoluteDH(d = D4)
    rob = DHRobot([e1,e2,e3,e4], name = 'RRPR')
    print(rob)
    print(rob.fkine(q))
    #print(rob.fkine_all(q))
    rob.teach(q)
    '''
    | q1  │  D1 │ L1 │   0.0° │ -180.0° │ 180.0° │
    │ q2  │   0 │ L2 │ 180.0° │ -180.0° │ 180.0° │
    │0.0° │  q3 │  0 │   0.0° │     0.0 │    1.0 │
    │ q4  │  D4 │  0 │   0.0° │ -180.0° │ 180.0° '''
    #0T4 =(0T4)^-1 = 0T1*1T2 *2T3 *3T4
    T01 = np.matrix(
        [[m.cos(q[0]),-m.sin(q[0])*m.cos(0),m.sin(q[0])*m.sin(0),L1*m.cos(q[0])],
         [m.sin(q[0]),m.cos(q[0])*m.cos(0),-m.cos(q[0])*m.sin(0),L1*m.sin(q[0])],
         [0,m.sin(0),m.cos(0),D1],
         [0,0,0,1]])
    T12 = np.matrix(
        [[m.cos(q[1]),-m.sin(q[1])*m.cos(PI),m.sin(q[1])*m.sin(PI),L2*m.cos(q[1])],
         [m.sin(q[1]),m.cos(q[1])*m.cos(PI),-m.cos(q[1])*m.sin(0),L2*m.sin(q[1])],
         [0,m.sin(PI),m.cos(PI),0],
         [0,0,0,1]])
    T23 = np.matrix(
        [[m.cos(0),-m.sin(0)*m.cos(0),m.sin(0)*m.sin(0),0],
         [m.sin(0),m.cos(0)*m.cos(0),-m.cos(0)*m.sin(0),0],
         [0,m.sin(0),m.cos(0),q[2]],
         [0,0,0,1]])
    T34 = np.matrix(
        [[m.cos(q[3]),-m.sin(q[3])*m.cos(0),m.sin(q[3])*m.sin(0),0],
         [m.sin(q[3]),m.cos(q[3])*m.cos(0),-m.cos(q[3])*m.sin(0),0],
         [0,m.sin(0),m.cos(0),D4],
         [0,0,0,1]])
    
    '''print(T01)
    print(T12)
    print(T23)
    print(T34)'''
    T= (np.dot(T01,T12))
    #print(T)
    T = np.dot(T,T23)
    #print(T)
    T04 = np.around(np.dot(T,T34),2)
    print(T04)
