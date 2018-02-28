clear;
clc;
close;
% Example of car moving forward in a grid, where it can move left or right
% as well. x1 is the vertical distance to an intersection and x2 is the 
% horizontal distance from the center. The car can either move forward
% (increase x1 by 1) and move side to side. At x1=0, there is a signal. 
% 
A = [0 0 ;
     0 0];
Bu = [1 0;
      0 1];

Bw = [0 0;
      0 0]; 

C = [0 0];
Du = [0 0];
Dw = [0 0];

Sys= STLC_lti(A,Bu,Bw,C,Du,Dw); 

Sys.x0= [-3 ; 0];

Sys.time = 0:1:100; 
Sys.ts=2; % sampling time for controller
Sys.L=10;   % horizon

Sys.u_ub = [1; 0];  % upper bound on u 
Sys.u_lb = [0; 0]; % lower bound on u
Sys.u_delta = [1; 1]; % not really sure what this does

Sys.Wref = [Sys.time*0.; Sys.time*0.];
% signal is default off
Sys.Wref(1,:) = -1;
% signal is on for some period of time
Sys.Wref(1,11/1:15/1) = 1;

Sys.stl_list = {};
% make sure we eventually pass the intersection
Sys.stl_list(end+1) = {'ev (x1(t)>1)'};
% make sure we aren't in the intersection while light is on
Sys.stl_list(end+1) = {'and alw (w1(t)>0 => (abs(x1(t)) > 1 and ev (w1(t) < 0)))'};

Sys.stl_list = {strjoin(Sys.stl_list)};


fprintf("getting controller...\n");
controller = get_controller(Sys)
%Sys.plot_x =[]; % default was Sys.plot_x = [1 2]

fprintf("running...\n");
%Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
Sys= Sys.run_open_loop(controller);