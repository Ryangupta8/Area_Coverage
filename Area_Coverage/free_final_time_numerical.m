function free_final_time_numerical
% The problem is formulated as a BVP and solved with bvp4c numerically

Dx=10;
Dy=10;

%%% First thing is to get our expressions for u1, u2 using dh/du1 = 0 and
%%% dh/du2 = 0 
% State equations
syms x1 x2 x3 x4 p1 p2 p3 p4 u1 u2;
Dx1 = x3;
Dx2 = x4;
Dx3 = u1;
Dx4 = u2;

% Cost function inside the integral
syms g;
w=0.5;
g = (1-w) + w*(u1^2 + u2^2);

% Hamiltonian
syms H;
H = g + p1*Dx1 + p2*Dx2 + p3*Dx3 + p4*Dx4;

% solve for control u
du1 = diff(H,u1);
du2 = diff(H,u2);
sol_u1 = solve(du1, u1)
sol_u2 = solve(du2, u2)
% Now we have the expressions u1 = -p3, u2 = -p4
%------------------------------------

solinit = bvpinit(linspace(0,1),[2;0.1;1;0;0;0;5;0;10]);

sol = bvp4c(@ode, @bc, solinit);
y = sol.y;
time = y(9)*sol.x;
u1t = -y(7,:);
u2t = -y(8,:);

figure(1);
plot(time,y(1,:)); hold on;
plot(time,y(2,:));
plot(time,y(3,:));
plot(time,y(4,:));
plot(time,u1t,'k:');
plot(time,u2t);
legend('x1(t)','x2(t)','x3(t)','x4(t)','u1(t)','u2(t)');
xlabel('time');
ylabel('states');
title('Numerical solution');
hold off;
print -djpeg90 -r0 final_free_statesVtime_1slice_numerical.jpg

figure(2);
plot(y(1,:),y(2,:),'-.r'); hold on;
rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle',':');
box on;
xlim([-1 11]);
ylim([-1 11]);
title('Robot Coverage Path - No Obstacles');
xlabel('x_1(t)');
ylabel('x_2(t)');
print -djpeg90 -r0 final_free_area_coverage_numerical.jpg
% -------------------------------------------------------------------------
% ODE's of augmented states
function dydt = ode(t,y)
dydt = y(9)*[ y(3);y(4);-y(7);-y(8);0;0;-y(5);-y(6);0 ];

% -------------------------------------------------------------------------
% boundary conditions: x1(0)=0;x2(0)=0.1,x3(0)=1,x4(0)=0
%                      x1(tf)=10, x2(tf)=0;
%                      p1(tf)*x2(tf)-0.5*p2(2)^2
function res = bc(ya,yb)
res = [ ya(1); ya(2)-0.1; ya(3)-1; ya(4); yb(1)-10; yb(2)-0.1; yb(3)-1; yb(4);
        0.5 + 0.5*(yb(7)^2 + yb(8)^2) + yb(5)*yb(3) + yb(6)*yb(4) - yb(7)^2 - yb(8)^2];
    
 %%% Side Note: If x2(tf) were free, rather than just tf being free, then
 %%% we can say as a BC that p2(tf)=0 based on the optimal control theory.
 %%% We also know that in the free final time case that H(tf)=0, hence the
 %%% last line of this res function
    
    
    
    
    
    
    