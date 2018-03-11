%%

close all;
clear;
clc;

%%
J = 0.000658;
C = 0.0000024571;
b = C;
K = 0.0787;
R = 5;
L = 0.58;

u = 5;

sys = tf(K, [J*L L*b+J*R R*b+K^2]);

kP = 200;
kI = 10;
kD = 4;
P = pid(kP, kI, kD);
PIDTF = feedback(P*sys, 1);

step(PIDTF, 0:.005:1);
title("PID Step Response");