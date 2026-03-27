clc;
clear all;

syms d1 a2 d4 d6


pose = ObtainPoseSym()

P = pose(1:3,4);
Pw = pose(1:3,3) * d6

pa = simplify(P - Pw)


