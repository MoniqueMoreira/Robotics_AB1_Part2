# Robotica AB1_Parte 2: Cinematica Direta 

## 1 Questão:
  Modelando a Manipulador RR Planar, atraves a biblioteca RoboticsToolBox do Peter Corke:
  ```
  Rob = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)
  ```
  Onde teremos 2 elos, ligados por uma uma junta do tipo Revolução, sendo L1 a distancia da junta 1 até a junta 2 e L2 a distancia da junta 2 até o Efetuador, sendo $`\theta 1`$ e $`\theta 2`$ os angulos da junta 1 e 2 respectivamente. Efetuando a cinematica inversa podemos encontra o ângulo(Em radianos) para a pose final dpo atuador P = $`[0.5,0.5]^T`$, atravez das equações:
```
B = m.acos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
A = m.atan2(y, x) - m.atan2(L2 * m.sin(B), L1 + L2* m.cos(B))
print(A,B)
```
**Saida:**
``` 
~ -0.4240310394907405 2.4188584057763776
```
```
print(f"Fkine =\n{Rob.fkine(q =[A,B])}")
```
**Saida:**
```
~ Fkine =
  -0.4114   -0.9114    0.5     
   0.9114   -0.4114    0.5     
   0         0         1
```

Pdemos interagir com o braço robotico criado chando a função teach(q), sendo q um array contendo ângulos(ou distancia no caso prismatico) da junta 1 e 2:

```
Rob.teach(q = [A,B])
```
<p align="center">
  <a name="figura-1"></a>
  <img src="Q1.png" alt="Manipulador RR Planar (RoboticsToolBox)" width="50%">
</p>

### Letra A
```
print(f"Fkine =\n{Rob.fkine(q =[-0.4240,2.4188])}")
Rob.teach(q =[-0.4240,2.4188])
print(Rob.fkine(q =[-0.4240,2.4188]).printline())
```
**Saida:**
```
Fkine =
  -0.4114   -0.9114    0.5     
   0.9114   -0.4114    0.5     
   0         0         1       

t = 0.5, 0.5; 114°
```
```
print(f"Fkine =\n{Rob.fkine(q =[1.9948,-2.4188])}")
Rob.teach(q =[1.9948,-2.4188])
print(Rob.fkine(q =[1.9948,-24188]).printline())
```
**Saida:**
```
Fkine =
   0.9115    0.4114    0.5     
  -0.4114    0.9115    0.5     
   0         0         1       

t = -0.85, 0.0128; -116°
```
<div style="display: flex;">
  <a name="figura-2"></a>
  <img src=Q1A1.png alt="q+" style="width: 47%;">
  <a name="figura-3"></a>
  <img src=Q1A2.png alt="q-" style="width: 45%;">
</div>

Podemos que em ambos os casos quando apartir do conjunto de ângulos q+ e q- levam nosso efetuador para para a mesma pose, istó se da pelo fato de Fkine ser não bijetiva, onde mais de um conjunto de q pode levar para mesmo ponto como visto na [q+](#figura-2) e na [q-](#figura-3) que estão no ponto (0.5.0.5) no plano. 

### Letra B

Modificando agora nosso braço para L1 = 2 e L2 = 1.5 e adicionado uma Prismatica de comprimento de 0 a 2, temos:
```
L1 = 2
L2 = 1.5
Rob = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)* ET2.tx(qlim=[0,2])
Rob.teach(q = [A,B,1])
Rob.teach(q = [0,B,2])
```
<div style="display: flex;">
  <a name="figura-4"></a>
  <img src=Q1B.png alt="q = [A,B,1]" style="width: 47%;">
  <a name="figura-5"></a>
  <img src=Q1B2.png alt="q = [0,B,2]" style="width: 45%;">
</div>

### Letra C

Com auxilio da função Fkine  e Fkine().printline() podemos calcular a transformação da base para o efetuador e a pose  do braço passando os angulos/distancias de cada junta.

```
q = [0,0.5,0.5]
print(Rob.fkine(q))
```
**Saida:**
```
Fkine =
   0.8776   -0.4794    3.755   
   0.4794    0.8776    0.9589  
   0         0         1       
```
```
print(Rob.fkine(q]).printline())
```
**Saida:**
```
t = 3.76, 0.959; 28.6°
```
```
Rob.teach(q)
```
<p align="center">
  <a name="figura-6"></a>
  <img src="Q1C.png" alt="q = [0,0.5,0.5] " width="50%">
</p>

## 2 Questão:


## 3 Questão:
