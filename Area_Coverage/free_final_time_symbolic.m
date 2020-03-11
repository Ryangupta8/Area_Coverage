function free_final_time_symbolic
clc
clear all
close all 

%Robot/Area Dimensions-----------------------------------------------------------
w = 0.5;                                        % weighing constant
radr = 0.1;                                     % radius of coverage

%Field Dimensions
Dx = 10;
Dy = 10;

%-------------------------------------------------------------------------- 
% Performance Parameter Initialization 
% area_iter(1) = 0;
time_tot = 0;
energy_tot = 0;
y_area_prev = 0;

x1_var = [];
x2_var = [];
x3_var = [];
x4_var = [];
u1_var = [];
u2_var = [];
pass_time = [];
energy = [];
% Generate Obstacles ------------------------------------------------------
% figure(1)
% 
% rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle','|');
% box on;
% xlim([-1 11]);
% ylim([-1 11]);
% title('Robot Coverage Path - No Obstacles');
% xlabel('X co-ordinate: State Variable, x_1');
% ylabel('Y co-ordinate: State Variable, x_2');
% hold on;
% pause(0.001);

% ---------------------------------------------------------------------------
% Supply ODE's and BCs on states and costates to dsolve
% Now, tf is itself a variable

% State equations
syms x1 x2 x3 x4 p1 p2 p3 p4 u1 u2;
Dx1 = x3;
Dx2 = x4;
Dx3 = u1;
Dx4 = u2;

% Cost function inside the integral
syms g;
g = (1-w) + w*(u1^2 + u2^2);

% Hamiltonian
syms H;
H = g + p1*Dx1 + p2*Dx2 + p3*Dx3 + p4*Dx4;

% Costate equations
Dp1 = -diff(H,x1);
Dp2 = -diff(H,x2);
Dp3 = -diff(H,x3);
Dp4 = -diff(H,x4);

% solve for control u
du1 = diff(H,u1);
du2 = diff(H,u2);
sol_u1 = solve(du1, u1);
sol_u2 = solve(du2, u2);


% Substitute u to state equations
Dx3 = subs(Dx3, u1, sol_u1);
Dx4 = subs(Dx4, u2, sol_u2);

% convert symbolic objects to strings for using 'dsolve'
eq1 = strcat('Dx1=',char(Dx1))
eq2 = strcat('Dx2=',char(Dx2))
eq3 = strcat('Dx3=',char(Dx3))
eq4 = strcat('Dx4=',char(Dx4))
eq5 = strcat('Dp1=',char(Dp1))
eq6 = strcat('Dp2=',char(Dp2))
eq7 = strcat('Dp3=',char(Dp3))
eq8 = strcat('Dp4=',char(Dp4))

sol_h = dsolve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8); 

%%%%%% HERE IS WHERE THEIR GYPSY MAGIC OCCURS
conA1 = 'x1(0) = 0';
conA2 = 'x2(0) = 0.1';
conA3 = 'x3(0) = 1';
conA4 = 'x4(0) = 0';
conA5 = 'x1(tf) = 10';
conA6 = 'x2(tf) = 0.1'; % i where: for i = radr:2*radr:Dx-radr <- is the overall loop 
conA7 = 'x3(tf) = 1';
conA8 = 'x4(tf) = 0';

sol_a = dsolve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8,...
               conA1,conA2,conA3,conA4, conA5, conA6, conA7, conA8);
           
eq1a = subs(sol_a.x1) - 'x1tf';
eq2a = subs(sol_a.x2) - 'x2tf';
eq3a = subs(sol_a.x3) - 'x3tf';
eq4a = subs(sol_a.x4) - 'x4tf';
eq5a = subs(sol_a.p1) - 'p1tf';
eq6a = subs(sol_a.p2) - 'p2tf';
eq7a = subs(sol_a.p3) - 'p3tf';
eq8a = subs(sol_a.p4) - 'p4tf';
eq9a = str2sym('p1tf*x2tf-p2tf*x2tf-0.5*p2tf^2');

sol_2 = solve(eq1a, eq2a, eq3a, eq4a, eq5a, eq6a, eq7a, eq8a, eq9a);
tf = sol_2.tf;
x1tf = sol_2.x1tf;
x2tf = sol_2.x2tf;
clear t;
x1 = subs(sol_a.x1);
x2 = subs(sol_a.x2);
x3 = subs(sol_a.x3);
x4 = subs(sol_a.x4);
p1 = subs(sol_a.p1);
p2 = subs(sol_a.p2);
p3 = subs(sol_a.p3);
p4 = subs(sol_a.p4);


