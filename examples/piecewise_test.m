clear;
clc;
close;
% Example of car moving forward in a grid, where it can move left or right
% as well. x1 is the vertical distance to an obstacle and x2 is the 
% horizontal distance from the obstacle. The car can either move forward
% (increase x1 by 1) and move side to side. 
% 
A = [1];
Bu = [0];

Bw = [0]; 

C = [0];
Du = [0];
Dw = [0];

Sys= STLC_lti(A,Bu,Bw,C,Du,Dw); 

Sys.x0= [.1];

Sys.time = 0:.1:40; 
Sys.ts=.1; % sampling time for controller
Sys.L=5;   % horizon

Sys.u_ub = [.1];  % upper bound on u 
Sys.u_lb = [0]; % lower bound on u
% Sys.u_delta = [1; 1]; % not really sure what this does

Sys.Wref = [Sys.time*0.];


Sys.stl_list = {'ev (x1(t) > 0)'};

fprintf("getting controller...\n");
controller = get_controller(Sys)
%Sys.plot_x =[]; % default was Sys.plot_x = [1 2]

fprintf("running...\n");
%Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
Sys= Sys.run_deterministic(controller);

