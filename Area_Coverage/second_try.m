function second_try
%SECONDTRY_bvp is second attempt at solving the optimal control problem  
%for area coverage using bvp4c
%  The problem is
%   
%      x1' = x3
%      x2' = x4
%      x3' = u1
%      x4' = u2
%   
%   The interval is [0 tf] and the boundary conditions are
%   
%      x1(0) = x2(0) = x3(0) = x4(0) = 0,  x1(tf) = 10, x2(tf) = x3(tf) = x4(tf) = 0
%  
%   The example uses a guess for the solution coded here in EX1INIT.

solinit = bvpinit(linspace(0,1,5),@second_try_init);
% A guess for a mesh that reveals the behavior of the solution is provided as the
% vector solinit.x. A guess for the solution at these mesh points is provided
% as the array solinit.y, with each column solinit.y(:,i) approximating the
% solution at the point solinit.x(i). It is not difficult to form a guess structure,
% but a helper function bvpinit makes it easy in the most common circumstances
options = bvpset('Stats','on','RelTol',1e-5);

sol = bvp4c(@ex1ode,@ex1bc,solinit,options);
% The mesh determined by
% the code is returned as sol.x and the numerical solution approximated at these
% mesh points is returned as sol.y. As with the guess, sol.y(:,i) approximates
% the solution at the point sol.x(i)




function dydx = ex1ode(x,y)
%EX1ODE ODE function for Example 1 of the BVP tutorial.
%   The components of y correspond to the original variables
%   as  y(1) = x1, y(2) = x2, y(3) = x3, y(4) = x4.

dydt =  [ y(3)
          y(4)
          u(1)
          u(2) ];

%-------------------------------------------------------------------------

function res = ex1bc(ya,yb)
%EX1BC Boundary conditions for Example 1 of the BVP tutorial.
%   RES = EX1BC(YA,YB) returns a column vector RES of the
%   residual in the boundary conditions resulting from the
%   approximations YA and YB to the solution at the ends of 
%   the interval [a b]. The BVP is solved when RES = 0. 
%   The components of y correspond to the original variables
%   as  y(1) = x1, y(2) = x3, y(3) = x3, y(4) = x4.

% In my case I have BCs for each of the variables, but no final time????

res = [ ya(1) - 1
        ya(2) - 1
        ya(3) - 1
        ya(4) + 10
        yb(3) - yb(5)];

%-------------------------------------------------------------------------

function v = second_try_init(x)
%EX1INIT guess for Example 1 of the BVP tutorial.

v = [        1 
             1
     -4.5*x^2+8.91*x+1
            -10 ];