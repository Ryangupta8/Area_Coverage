function n_slice_problem
clc
close all
clear all 
global counter


    
    global counter
    counter = 0;
    
    %Robot/Area Dimensions-----------------------------------------------------------
    w = 0.5;                                        % weighing constant
    radr = 0.2;                                     % radius of coverage
    
    %Field Dimensions
    Dx = 10;
    Dy = 10;

    % Performance Parameter Initialization ------------------------------------
    time_tot = 0;
    energy_tot = 0;
    cost_tot = 0;
    y_area_prev = 0;

    x1_var = [];
    x2_var = [];
    x3_var = [];
    x4_var = [];
    u1_var = [];
    u2_var = [];
    pass_time = [];
    energy = [];
    
    % Area Coverage Plot Initialization -------------------------------------------------
    % Want to generate the figure that is the [Dx, Dy] Grid. 
    %   This figure will display the trajectories.
    figure(1);
    rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle',':');
    box on;
    xlim([-1 11]);
    ylim([-1 11]);
    title('Robot Coverage Path - No Obstacles');
    xlabel('X State Variable, x_1');
    ylabel('Y State Variable, x_2'); hold on;
    % print -djpeg90 -r0 final_free_area_coverage_numerical.jpg
    
    
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
    sol_u1 = solve(du1, u1);
    sol_u2 = solve(du2, u2);
    % Now we have the expressions u1 = -p3, u2 = -p4
    
    %-------------------------------------------------------------------------------
    % Now to begin the loop. We are solving the tpbvp for each 'sweep'
    %   Will need to keep track of:
    %       - total energy
    %       - total time 
    %       - etc?
    for i = radr:2*radr:Dx-radr
        % Round iteration to nearest integer
        iter = round((i-radr)/(2*radr));
        
        % Define the initial guess
        %       [ x1; x2; x3; x4; p1; p2; p3; p4; H]
        guess = [ 2; iter*radr; 0; 0; 0; 0; 5*(-1)^(iter); 0; 10 ];
        
        % init the boundary value solver bvp4c
        solinit = bvpinit(linspace(0,1),guess);
        
        if rem(iter,2) == 0 % if we are in an even iteration
            sol_ = bvp4c(@ode, @bceven, solinit);
            y = sol_.y;
            time = y(9)*sol_.x;
            u1t = -y(7,:);
            u2t = -y(8,:);
            plot(y(1,:),y(2,:),'-.r');
        else
            sol_ = bvp4c(@ode, @bcodd, solinit);
            y = sol_.y;
            time = y(9)*sol_.x;
            u1t = -y(7,:);
            u2t = -y(8,:);
            plot(y(1,:),y(2,:),'-.r');
        end
        % Keep track of the cost by step and overall 
%         step_cost = symsum(w + (1 - w)*(u1t^2 + u2t^2),sol_.t,0,time)
%         
%         cost_tot = cost_tot + step_cost; 
        % Keep track of total time
        time_tot = time_tot + time; 
%         % Keep track of energy total
%         energy_tot = energy_tot + 0.5*(u1t^2 + u2t^2);
        plot(y(1,:),y(2,:),'-.r');
        clear y u1t u2t time step_cost;
%         counter*0.2
        counter = counter + 1
    end
    
    print -djpeg90 -r0 n_slice_area_coverage.jpg
end

% -------------------------------------------------------------------------
% ODE's of augmented states
function dydt = ode(t,y)
    dydt = y(9)*[ y(3);y(4);-y(7);-y(8);0;0;-y(5);-y(6);0 ];  
end
        
       
% -------------------------------------------------------------------------
% boundary conditions: x1(0)=0;x2(0)=0.2+counter*radr;x3(0)=1;x4(0)=0
% odd iterations       x1(tf)=10;x2(tf)=0.2+counter*radr;x3(tf)=1;x4(tf)=0
% Direction ---->      0.5 + 0.5*(p3(tf)^2 + p4(tf)^2) + ...
%                      p1(tf)*x3(tf) + p2(tf)*x4(tf) - p3(tf)^2 - p4(tf)^2   
function res = bcodd(ya,yb) 
 global counter
 res = [ ya(1); ya(2)-0.2-2*counter*0.2; ya(3)-1; ya(4); yb(1)-10; yb(2)-0.2-2*counter*0.2; yb(3)-1; yb(4);
        0.5 + 0.5*(yb(7)^2 + yb(8)^2) + yb(5)*yb(3) + yb(6)*yb(4) - yb(7)^2 - yb(8)^2];       
        
end
% boundary conditions: x1(0)=10;x2(0)=0.2+counter*radr;x3(0)=-1;x4(0)=0
% even iterations      x1(tf)=0;x2(tf)=0.2+counter*radr;x3(tf)=-1;x4(tf)=0
% Direction <----      0.5 + 0.5*(p3(tf)^2 + p4(tf)^2) + ...
%                      p1(tf)*x3(tf) + p2(tf)*x4(tf) - p3(tf)^2 - p4(tf)^2
function res = bceven(ya,yb) 
 global counter
 res = [ ya(1)-10; ya(2)-0.2-2*counter*0.2; ya(3)+1; ya(4); yb(1); yb(2)-0.2-2*counter*0.2; yb(3)+1; yb(4);
        0.5 + 0.5*(yb(7)^2 + yb(8)^2) + yb(5)*yb(3) + yb(6)*yb(4) - yb(7)^2 - yb(8)^2];
        
        
end




