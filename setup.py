"""
Inicializa o workspace do robotics toolbox no iPython.
"""

import numpy as np
from roboticstoolbox import ET2, ET, ERobot, DHRobot, RevoluteDH, PrismaticDH, models


PLOT = True
PI = np.pi


def ets_2d():
    """
    Cria um manipulador planar do tipo RR.
    """
    print("# --- Exemplo simples ETS 2D --- #")
    a_1 = 1
    a_2 = 1
    robot_2 = ET2.R() * ET2.tx(a_1) * ET2.R() * ET2.tx(a_2)

    print(f"Fkine =\n{robot_2.fkine(np.deg2rad([30, 40]))}")
    print(f"# Joints: {robot_2.n}")
    print(f"Joints: {robot_2.joints()}")
    print(f"Structure: {robot_2.structure}")

    robot_2.teach(q=np.deg2rad([30, 40]))

def ets_3d():
    """
    Cria um manipulador espacial do tipo RRRRRR.
    """
    print("# --- Exemplo simples ETS 3D --- #")
    a_1 = 1
    a_2 = 1
    robot6 = ERobot(
        ET.Rz() * ET.Ry() * ET.tz(a_1) * ET.Ry() * ET.tz(a_2) \
        * ET.Rz() * ET.Ry() * ET.Rz(), name="robot6"
    )

    print(f"Fkine =\n{robot6.fkine(np.deg2rad([30, 40, 50, 60, 70, 80]))}")
    print(f"# Joints: {robot6.n}")
    print(f"Structure: {robot6.structure}")

    robot6.teach(q=np.deg2rad([30, 40, 50, 60, 70, 80]))


def get_models(model_type="ETS"):
    """
    Lista os modelos disponíveis.
    """
    if model_type == "ETS":
        print("# --- Lista de modelos ETS --- #")
        print(models.list(type="ETS"))
    elif model_type == "DH":
        print("# --- Lista de modelos DH --- #")
        print(models.list(type="DH"))
    else:
        raise ValueError("Tipo de modelo inválido.")


def panda_robot():
    """
    Cria um manipulador do tipo Panda.
    """
    print("# --- Exemplo com um manipulador de 7 graus de liberdade --- #")
    panda = models.ETS.Panda()
    print(panda)
    print(panda.fkine(panda.qr).printline()) # imprime a trans. homo. de maneira mais legível
    print(panda.fkine_all(panda.qr))
    panda.plot(panda.qr) # visualiza o manipulador sem interação

def Pl_robot():
    """
    Cria um manipulador do tipo Panda.
    """
    print("# --- Exemplo com um manipulador de 7 graus de liberdade --- #")
    panda = models.DH.AL5D()
    print(panda)
    print(panda.fkine(q=[0, 0, 0])) # imprime a trans. homo. de maneira mais legível
    panda.teach(q=[0, 0, 0]) # visualiza o manipulador sem interação


def dh_sample():
    """
    Cria um manipulador do tipo RRP.
    """
    print("# --- Exemplo utilizando DH --- #")

    link1 = RevoluteDH(a=1) # cria um link de revolução
    link2 = PrismaticDH(theta=PI/2, alpha=PI/2, qlim=(0., 1.)) # cria um link prismatico
    link3 = RevoluteDH(a=1)

    print(link1.A(0.5)) # calcula a matriz de transformação homogênea para theta=0.5

    robot_dh = DHRobot([link1, link2, link3], name="RRP") # cria um manipulador DH

    print(robot_dh)
    print(robot_dh.fkine(q=[0, 0.5, 0]))

    robot_dh.teach(q=[0, 0.5, 0]) # visualiza o manipulador com interação
