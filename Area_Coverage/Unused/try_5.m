function try_5
clc
clear all
close all

%Robot/Area Dimensions-----------------------------------------------------------
w = 0.5;                                        % weighing constant
radr = 0.1;                                     % radius of coverage

%Field Dimensions
Dx = 10;
Dy = 10;


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
%%% "We first supply the ODE?s and boundary... 
%%%         conditions on states and costates to dsolve" 
%------------------------------------------------------------------------------------
% use boundary conditions to determine the coefficients
conA1 = 'x1(0) = 0';
conA2 = 'x2(0) = 0.1';
conA3 = 'x3(0) = 1';
conA4 = 'x4(0) = 0';
conA5 = 'x1(2) = 10';
conA6 = 'x2(2) = 0.1'; % i where: for i = radr:2*radr:Dx-radr <- is the overall loop 
conA7 = 'x3(2) = 1';
conA8 = 'x4(2) = 0';

sol_a = dsolve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8,...
               conA1,conA2,conA3,conA4, conA5, conA6, conA7, conA8);


time = linspace(0,2,20);

% plot both solutions
figure(1);
fplot(sol_a.x1,[0 2]); hold on;
fplot(sol_a.x2,[0 2]);
fplot(sol_a.x3,[0 2]);    
fplot(sol_a.x4,[0 2]);    
fplot(-sol_a.p3,[0 2]);    % plot the control: u1=-p3
fplot(-sol_a.p4,[0 2]);    % plot the control: u2=-p4

legend('x1(t)','x2(t)','x3(t)','x4(t)','u1(t)','u2(t)')
axis([0 2 -5 12]);
text(0.2,1.1,'x_1(t)');
text(0.6,0.5,'x_2(t)');
text(0.2,4.0,'x_3(t)');
text(0.2,-1.0,'x_4(t) = 0');
text(0.5,-1.0,'u_2(t) = 0');
text(0.45,8.0,'u_1(t)');
xlabel('time');
ylabel('states');
hold off;
print -djpeg90 -r0 statesVtime_1slice.jpg

figure(2);
fplot(sol_a.x1,sol_a.x2,[0,2],'-.r'); hold on;
rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle',':');
box on;
xlim([-1 11]);
ylim([-1 11]);
title('Robot Coverage Path - No Obstacles');
xlabel('x_1(t)');
ylabel('x_2(t)');
print -djpeg90 -r0 area_coverage.jpg



end