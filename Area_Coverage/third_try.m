%%% The first step is to form the Hamiltonian and apply the necessary
%%% conditions for optimality 

% For this run I say x0 y0 is 3,3
x0 = 3;
y0 = 3;
r0 = 0.5;
w = 0.1;
% State equations 
syms x1 x2 x3 x4 x5 p1 p2 p3 p4 p5 u1 u2 w; 
Dx1 = x3;
Dx2 = x4;
Dx3 = u1;
Dx4 = u2;
Dx5 = 1;

% Cost function inside the integral
syms g;
g = ((1-w)*(u1^2 + u2^2) + w);

% Hamiltonian
syms H;
H = g + p1*Dx1 + p2*Dx2 + p3*Dx3 + p4*Dx4 + p5*Dx5; 
%p5*((x1)^2 * (-x1) + (10 - x1)^2 * (x1-10) + (x2)^2 * (-x2) + (10 - x2)^2 * (x2-10) + ((x1 - x0)^2 + (x2 - y0)^2 - r0^2)^2)

% Costate equations 
Dp1 = -diff(H,x1); 
Dp2 = -diff(H,x2);
Dp3 = -diff(H,x3);
Dp4 = -diff(H,x4);
Dp5 = 0;

% solve for control u 
du1 = diff(H,u1)
% sol_u1 = solve(du1, 'u1');
du2 = diff(H,u2)
sol_u2 = solve(du2, 'u2');
% The MATLAB commands we used here are diff and solve.
% diff differentiates a symbolic expression and solve gives symbolic solution 
% to algebraic equations


%%% The second step is to substitute u from (4 - that is partialH/partialu = 0) back to the state and costate 
%%% equations to get a set of 2n first-order ordinary differential equations (ODE?s). 
%%% A solution (with 2n arbitrary coefficients) can be obtained by using 
%%% the dsolve command without any boundary conditions

% Substitute u to state equations 
Dx3 = subs(Dx3, u1, sol_u1);
Dx4 = subs(Dx4, u2, sol_u2);

% % convert symbolic objects to strings for using ?dsolve?
% eq1 = strcat('Dx1=',char(Dx1));
% eq2 = strcat('Dx2=',char(Dx2));
% eq3 = strcat('Dp1=',char(Dp1));
% eq4 = strcat('Dp2=',char(Dp2)); sol_h = dsolve(eq1,eq2,eq3,eq4);



