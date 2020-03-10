%------------ Optimal Robot Path Planning ----------------------------------------------
% Code:        Coverage Path Planning Algorithm - No Obstacles
% 
% Description  The following code consists of a coverage path planning algorithm 
%              implemented using pseudospectral optimal control and through boustro-
%              phedon cellular decomposition for a rectangular field with no obstacles 
%              and assuming a point robot with a fixed coverage radius.
%---------------------------------------------------------------------------------------
clc
close all
clear all

% Generate States and Control Variables ------------------------------------------------
syms x1(t) x2(t) x3(t) x4(t);     % represents 2D physical coordinates as states
syms u1(t) u2(t);                 % Control inputs (accelerations)
syms Dx1 Dx2 Dx3 Dx4;

%Robot/Area Dimensions-----------------------------------------------------------
w = 0.5;                                        % weighing constant
radr = 0.1;                                     % radius of coverage

%Field Dimensions
Dx = 10;
Dy = 10;

%Obstacle Parameters ------------------------------------------------------ 
% Performance Parameter Initialization ------------------------------------
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
figure(1)

rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle',':');
box on;
xlim([-1 11]);
ylim([-1 11]);
title('Robot Coverage Path - No Obstacles');
xlabel('X co-ordinate: State Variable, x_1');
ylabel('Y co-ordinate: State Variable, x_2');
hold on;
pause(0.001);

for i = radr:2*radr:Dx-radr

iter = round((i-radr)/(2*radr));
% solution = ezsolve(objective, {cbox, cbnd, ceq}, x0 , options);
% constraints = {x+y<=1.9, x>=0, y>=0};
cbox = {0 <= x1(t) <= 10,... 
    i-2*radr+0.01 <= x2(t) <= i+2*radr};

cbdc = {x1(0) == 0, x2(0) == 0, x3(0) == 0, x4(0) == 0,...
        x1(tf) == 10, x2(tf) == i, x3(tf) == 1, x4(tf) == 0};
% x0 = { tf == 10 
%      icollocate({x1 == t; x2 == i; x3 == 1; x4 == 1})
%      collocate({u1 == 1; u2==1})};
% 
% cbox = { 0  <= collocate(x1) <= 10
%          i-2*radr+0.01 <= collocate(x2) <= i+2*radr};
% 
% cbnd = {initial({x1 == 0; x2 == i
%                  x3 == 1; x4 == 0})
%         final({  x1 == 10; x2 == i
%                  x3 == 1; x4 == 0})};
%--------------------------------------------------------------------------

% ODEs and path constraints------------------------------------------------

ceq = {                                      % state equations
    Dx1 == x3(t)
    Dx2 == x4(t)
    Dx3 == u1(t)
    Dx4 == u2(t)
     };

tot_cost = int(w + (1 - w)*(u1(t)^2 + u2(t)^2),t);

% Objective----------------------------------------------------------------
objective = tot_cost;
 
% Solution of Differential Equation Set------------------------------------
options = struct;
options.name = 'Optimal Robot Path Planning ';
% solution = ezsolve(objective, {cbox, cbnd, ceq}, x0 , options);
% t_p  = subs(icollocate(t),solution);
% x1_p = subs(icollocate(x1),solution);
% x2_p = subs(icollocate(x2),solution);
% x3_p = subs(icollocate(x3),solution);
% x4_p = subs(icollocate(x4),solution);
% u1_p = subs(icollocate(u1),solution);
% u2_p = subs(icollocate(u2),solution);
% 
% x1_var = cat(1,x1_var,x1_p);
% x2_var = cat(1,x2_var,x2_p);
% x3_var = cat(1,x3_var,x3_p);
% x4_var = cat(1,x4_var,x4_p);
% u1_var = cat(1,u1_var,u1_p);
% u2_var = cat(1,u2_var,u2_p);
% 
% area_iter(iter+1) = trapz(x2_p(2:length(x2_p)) +radr) - trapz(x2_p(2:length(x2_p)) -radr);
% time_iter(iter+1) = max(t_p);
% energy_iter(iter+1) = 0.1*trapz(sqrt(u1_p.^2+u2_p.^2));
% 
% plot(x1_p,x2_p,'b')
% 
% if i == 10-radr
%     break;
% elseif ((mod(round((i-radr)/(2*radr)),2)) == 0)  
%     plot([10 10]',[i i+2*radr]','r','LineWidth',1.5);
% elseif ((mod(round((i-radr)/(2*radr)),2)) == 1)
%     plot([0 0]',[i i+2*radr]','r','LineWidth',1.5);
% end
% 
% hold on;
% pause(0.001);
% 
% if i == radr
%     text(x1_p(1),x2_p(1), ' Initial Point \rightarrow', 'HorizontalAlignment', 'right');
% end

end