clear;

% Set environmental variables
x0 = 5; % x position of obstacle
y0 = 1; % y position of the obstacle
r0 = 1; % radius of the obstacle
w=1; % "Time" in the cost function 0 <= w <= 1


% State equations
syms x1 x2 x3 x4 p1 p2 p3 p4 p5 u1 u2 H;
Dx1 = x3;
Dx2 = x4;
Dx3 = u1;
Dx4 = u2;

% Cost function inside the integral
syms g;
g = ((1-w)*(u1^2 + u2^2) + w);

% Hamiltonian
H = g + p1*Dx1 + p2*Dx2 + p3*Dx3 + p4*Dx4 + p5*((x1)^2 * (-x1) + (10 - x1)^2 * (x1-10) + (x2)^2 * (-x2) + (10 - x2)^2 * (x2-10) + ((x1 - x0)^2 + (x2 - y0)^2 - r0^2)^2);

% Costate equations
Dp1 = -diff(H,x1);
Dp2 = -diff(H,x2);
Dp3 = -diff(H,x3);
Dp4 = -diff(H,x4);
Dp5 = 0;

% solve for control u
du1 = diff(H,u1);
du2 = diff(H,u2);
sol_u1 = solve(du1, u1);
sol_u2 = solve(du2, u2);

% Substitute u1, u2 to state equations
Dx3 = subs(Dx3, u1, sol_u1);
Dx4 = subs(Dx4, u2, sol_u2);

% % convert symbolic objects to strings for using ?dsolve?
% eq1 = strcat('Dx1=',char(Dx1));
% eq2 = strcat('Dx2=',char(Dx2));
% eq3 = strcat('Dx3=',char(Dx3));
% eq4 = strcat('Dx4=',char(Dx4));
% 
% eq5 = strcat('Dp1=',char(Dp1));
% eq6 = strcat('Dp2=',char(Dp2));
% eq7 = strcat('Dp3=',char(Dp3));
% eq8 = strcat('Dp4=',char(Dp4));
% sol_h = dsolve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8);

% conA1 = 'x1(0) = 0';
% conA2 = 'x2(0) = 0';
% conA3 = 'x1(w) = 10';
% conA4 = 'x2(w) = 0.2';
% conA5 = '(x1(w) - x1(0))^2 +(x2(w) - 0)^2 >= 0.2^2'; 
% sol_a = dsolve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8,conA1,conA2,conA3,conA4);
% 
% x = 0:0.1:5;
% % fplot(x, sol_a)
% eplot(x,sol_h.x1)


%%% BVP %%%

sol = dsolve('Dx1 = x3, Dx2 = x4, Dx3 = u1, Dx4 = u2, Dp1 = -diff(H,x1), Dp2 = -diff(H,x2), Dp3 = -diff(H,x3), Dp4 = -diff(H,x4)',...
'x1(0) = 0, x2(0) = 0, x3(0) = 0, x4(0) = 0, x1(tf) = 10, x2(tf) = 0, x3(tf) = 0, x4(tf) = 0, p2(tf) = 0');
eq1 = subs(sol.x1) - 'x1tf';
eq2 = subs(sol.x2) - 'x2tf';
eq3 = subs(sol.p1) - 'p1tf';
eq4 = subs(sol.p2) - 'p2tf';
eq5 = sym('p1tf*x2tf - 0.5*p2tf^2');

% sol_2 = solve(eq1, eq2, eq3, eq4, eq5);
% tf = sol_2.tf;
% x1tf = sol_2.x1tf;
% x2tf = sol_2.x2tf;
% x1 = subs(sol.x1);
% x2 = subs(sol.x2);
% p1 = subs(sol.p1);
% p2 = subs(sol.p2);

% % Initial guess for the solution
% solinit = bvpinit(linspace(0,0.78,50), ...
% [0 0 0.5 0.5]);
% options = bvpset('Stats','on','RelTol',1e-1);
% global R;
% R = 0.1;
% sol = bvp4c(@BVP_ode, @BVP_bc, solinit, options);
% t = sol.x;
% y = sol.y;
% % Calculate u(t) from x1,x2,p1,p2
% ut = (y(3,:).*(y(1,:) + 1/4))/(2*0.1);
% n = length(t);
% % Calculate the cost
% J = 0.78*(y(1,:)*y(1,:)? + y(2,:)*y(2,:)? + ...
% ut*ut?*0.1)/n;