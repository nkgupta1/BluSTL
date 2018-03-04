clear;
clc;
close;
% Example of car moving forward in a grid, where it can move left or right
% as well. x1 is the vertical distance to an obstacle and x2 is the 
% horizontal distance from the obstacle. The car can either move forward
% (increase x1 by 1) and move side to side. 
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

Sys.time = 0:.5:20; 
Sys.ts=1; % sampling time for controller
Sys.L=10;   % horizon

Sys.u_ub = [1; 1];  % upper bound on u 
Sys.u_lb = [0; -1]; % lower bound on u
% Sys.u_delta = [1; 1]; % not really sure what this does

Sys.Wref = [Sys.time*0.; Sys.time*0.];


Sys.stl_list = {'alw ( x2(t)<2 and x2(t)>-2 and (abs(x1(t)) > 0.4 or abs(x2(t)) > 0.4)) and ev (x1(t)>0) '};

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
set(gca, 'fontsize', 25)
rectangle('Position', [-0.4 -0.4 0.8 0.8], 'FaceColor', 'black')
for i=1:21
hold on;
    title(['time=' num2str(Sys.time(i))], 'fontsize', 30);
    h2 = plot( Sys.system_data.X(2, i),  Sys.system_data.X(1, i), '^r-', 'MarkerSize', 25, 'MarkerFaceColor', 'r');
    addpoints(h, Sys.system_data.X(2, i), Sys.system_data.X(1, i));
    drawnow;
    hold off;
    %pause(.1);
    %saveas(gcf, ['images/car-'  num2str(i) '.png']);
    set(h2, 'Visible', 'off');
end
set(h2, 'Visible', 'on');
saveas(gcf, ['images/car.png']);
