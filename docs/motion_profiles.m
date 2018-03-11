%% 

close all;
clear;
clc;

%%
% a_m = 2;
% j_m = 10;
syms a_m j_m v_m v_f v_0 d;

t_1 = a_m / j_m;
v_1 = v_0 + a_m^2 / (2 * j_m);
v_2 = v_m - a_m^2 / (2 * j_m);
t_2 = t_1 + (v_2 - v_1) / a_m;
t_m = t_2 + t_1;

t_3 = t_m + a_m / j_m;
v_3 = v_m - a_m^2 / (2*j_m);
v_4 = v_f + a_m^2 / (2*j_m);
t_4 = t_3 - (v_4 - v_3) / a_m;
t_f = t_4 + t_1;

p1 = t_1*v_0;
p2 = (t_2-t_1)*(v_2+v_1)/2;
p3 = (t_3-t_2)*v_2;
p4 = (t_4-t_3)*(v_3+v_4)/2;
p5 = (t_f-t_4)*v_f;
p6 = t_1*(v_1-v_0);
D = p1 + p2 + p3 + p4 + p5 + 2 * p6;
D = simplify(D);
disp("formula for displacement");
disp(D);
disp("displacement for a=2, j=10, v0=1, vf=0")
vars = [a_m j_m v_0 v_m v_f];
vals = [2 10 1 5 0];
disp(vpa(subs(D, vars, vals)));

% disp(diff(D, v_m));
