%% 

close all;
clear;
clc;

%% Computing Max Acheivable Velocity
% a_m = 2;
% j_m = 10;
syms a_1 a_2 j_m v_m v_f v_0 d;

t_1 = a_1 / j_m;
v_1 = v_0 + a_1^2 / (2 * j_m);
v_2 = v_m - a_1^2 / (2 * j_m);
t_2 = t_1 + (v_2 - v_1) / a_1;
t_m = t_2 + t_1;

t_3 = t_m + a_2 / j_m;
v_3 = v_m - a_2^2 / (2*j_m);
v_4 = v_f + a_2^2 / (2*j_m);
t_4 = t_3 - (v_4 - v_3) / a_2;
t_f = t_4 + t_1;

p1 = t_1*v_0;
p2 = (t_2-t_1)*(v_2+v_1)/2;
p3 = (t_3-t_2)*v_2;
p4 = (t_4-t_3)*(v_3+v_4)/2;
p5 = (t_f-t_4)*v_f;
p6 = t_1*(v_1-v_0);
D = p1 + p2 + p3 + p4 + p5 + 2 * p6;

D = simplify(D) - d;
disp("formula for displacement:");
pretty(D);
disp("displacement:");
disp(vpa(subs(D, [a_1 a_2 j_m v_0 v_m v_f d], [5 5 15 0 5 1 5])));

return

d_val = 5;
vars = [a_m j_m v_0 v_f];
vals = [2 10 1 0];
specific_D = subs(D, vars, vals);
a = 1/a_m;
b = a_m/j_m;
c = (a_m^2*(v_0+v_f) - j_m*(v_0^2+v_f^2))/(2*a_m*j_m) - d_val;
sln_eq = (-b + sqrt(b^2-4*a*c))/(2*a);
sln_v_m = vpa(subs(sln_eq, vars, vals));
disp("Constraining distance to:");
disp(d_val);
disp("Solving:");
disp(d_val == specific_D);
disp("Max speed to acheive d is:");
disp(sln_v_m);

%% Compute Max Acheivable Acceleration

D_acc = p1+p5+2*p6;
disp("formula for displacement of acc:");
disp(D_acc);
D_acc = simplify(D_acc);
disp("simplified formula for displacement of acc:");
disp(D_acc);
disp("Solving:");
disp(0 == D_acc - d);
a = 1/j_m^2;
b = (j_m*v_0 + j_m*v_f)/(j_m^2);
c = d;
sln_eq = (-b + sqrt(b^2-4*a*c))/(2*a);
disp("solving");
disp(sln_eq);
disp(vpa(subs(sln_eq, [v_0 v_f j_m d], [0 0 50 3])));