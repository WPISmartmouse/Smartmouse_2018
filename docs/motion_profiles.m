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
disp("formula for displacement:");
disp(D);
disp("displacement:");
disp(vpa(subs(D, [a_m j_m v_0 v_m v_f], [5 15 0 5 1])));

d = 5;
vars = [a_m j_m v_0 v_f];
vals = [2 10 1 0];
specific_D = subs(D, vars, vals);
a = 1/a_m;
b = a_m/j_m;
c = (a_m^2*(v_0+v_f) - j_m*(v_0^2+v_f^2))/(2*a_m*j_m) - d;
sln_eq = (-b + sqrt(b^2-4*a*c))/(2*a);
sln_v_m = vpa(subs(sln_eq, vars, vals));
disp("Constraining distance to:");
disp(d);
disp("Solving:");
disp(d == specific_D);
disp("Max speed to acheive d is:");
disp(sln_v_m);

% disp(diff(D, v_m));
