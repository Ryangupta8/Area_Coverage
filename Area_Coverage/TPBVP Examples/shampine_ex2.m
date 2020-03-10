function dydt = ex2ode(t,y)
p = 1e-5;
dydt = [ y(2)
-3*p*y(1)/(p+t^2)^2];

function ex1bvp
%EX1BVP  Example 1 of the BVP tutorial.
%   This is the example for MUSN in U. Ascher, R. Mattheij, and R. Russell, 
%   Numerical Solution of Boundary Value Problems for Ordinary Differential 
%   Equations, SIAM, Philadelphia, PA, 1995.  MUSN is a multiple shooting 
%   code for nonlinear BVPs.  The problem is
%   
%      u' =  0.5*u*(w - u)/v
%      v' = -0.5*(w - u)
%      w' = (0.9 - 1000*(w - y) - 0.5*w*(w - u))/z
%      z' =  0.5*(w - u)
%      y' = -100*(y - w)
%   
%   The interval is [0 1] and the boundary conditions are
%   
%      u(0) = v(0) = w(0) = 1,  z(0) = -10,  w(1) = y(1)
%   
%   The example uses a guess for the solution coded here in EX1INIT.  
%   The results of a run of the FORTRAN code MUSN are here compared to
%   the curves produced by BVP4C.  

solinit = bvpinit(linspace(0,1,5),@ex1init);
options = bvpset('Stats','on','RelTol',1e-5);

sol = bvp4c(@ex1ode,@ex1bc,solinit,options);

% The solution at the mesh points
% As with the guess, sol.y(:,i) approximates
% the solution at the point sol.x(i).
x = sol.x;
y = sol.y;

clf reset
plot(x,y','*')
axis([0 1 -0.5 2.5])
title('Shampine Example 1')
ylabel('bvp4c (*) solutions')
xlabel('x')
legend('sol.x', 'sol.y')
shg


function dydx = ex1ode(x,y)
%EX1ODE ODE function for Example 1 of the BVP tutorial.
%   The components of y correspond to the original variables
%   as  y(1) = u, y(2) = v, y(3) = w, y(4) = z, y(5) = y.

dydx =  [ 0.5*y(1)*(y(3) - y(1))/y(2)
         -0.5*(y(3) - y(1))
         (0.9 - 1000*(y(3) - y(5)) - 0.5*y(3)*(y(3) - y(1)))/y(4)
          0.5*(y(3) - y(1))
          100*(y(3) - y(5)) ];

%-------------------------------------------------------------------------

function res = ex1bc(ya,yb)
%EX1BC Boundary conditions for Example 1 of the BVP tutorial.
%   RES = EX1BC(YA,YB) returns a column vector RES of the
%   residual in the boundary conditions resulting from the
%   approximations YA and YB to the solution at the ends of 
%   the interval [a b]. The BVP is solved when RES = 0. 
%   The components of y correspond to the original variables
%   as  y(1) = u, y(2) = v, y(3) = w, y(4) = z, y(5) = y.

res = [ ya(1) - 1
        ya(2) - 1
        ya(3) - 1
        ya(4) + 10
        yb(3) - yb(5)];

%-------------------------------------------------------------------------

function v = ex1init(x)
%EX1INIT guess for Example 1 of the BVP tutorial.

v = [        1 
             1
     -4.5*x^2+8.91*x+1
            -10
     -4.5*x^2+9*x+0.91 ];
