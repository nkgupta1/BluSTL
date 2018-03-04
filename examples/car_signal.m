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

Sys.time = 0:1:70; 
Sys.ts=2; % sampling time for controller
Sys.L=10;   % horizon

Sys.u_ub = [1; 0];  % upper bound on u 
Sys.u_lb = [0; 0]; % lower bound on u
Sys.u_delta = [1; 1]; % not really sure what this does

Sys.Wref = [Sys.time*0.; Sys.time*0.];
% signal is default off
Sys.Wref(1,:) = -1;
% signal is on for some period of time
Sys.Wref(1,4/1:20/1) = 1;

Sys.stl_list = {};
% make sure we eventually pass the intersection
% Sys.stl_list(end+1) = {'ev (x1(t)>1)'};
% make sure we aren't in the intersection while light is on
Sys.stl_list(end+1) = {'alw (w1(t)>0 => (abs(x1(t)) > 1 and ev (w1(t) < 0)))'};
% make sure that we progress if the enviornment is good to us
Sys.stl_list(end+1) = {'and alw (w1(t)<0 => ev (x1(t)>1))'};

Sys.stl_list = {strjoin(Sys.stl_list)};


fprintf("getting controller...\n");
controller = get_controller(Sys)
%Sys.plot_x =[]; % default was Sys.plot_x = [1 2]

fprintf("running...\n");
%Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
Sys= Sys.run_deterministic(controller);

%% Plot the path of the car over time
figure;
h = animatedline('linewidth', 10);
axis([-3 3 -3 3]);
set(gca, 'fontsize', 25);
hold on;
rectangle('Position', [-3 -1 6 2], 'LineWidth', 3, 'FaceColor', [0 0 0 0.5]);
hold on;
for i=1:length(Sys.system_data.X(1, :))
    title(['time=' num2str(Sys.time(i))], 'fontsize', 30);
    h2 = plot( Sys.system_data.X(2, i),  Sys.system_data.X(1, i), '^r-', 'MarkerSize', 25, 'MarkerFaceColor', 'r');
    addpoints(h, Sys.system_data.X(2, i), Sys.system_data.X(1, i))
    drawnow;
    % pause(.1);
    % saveas(gcf, ['images/car_signal-'  sprintf('%03d', i) '.png']);
    set(h2, 'Visible', 'off');
end
set(h2, 'Visible', 'on')