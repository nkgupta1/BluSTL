clear;
clc;
close;
% Example of car moving forward in a grid, and trying to make a left turn
% on a yield. There is oncoming traffic. We kinda cheat by allowing the car
% to move in a direction without explictly turning. This is a
% simplification until integrating piecewise linear works.
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

% x, y
Sys.x0= [0.5 ; -3];

max_time = 20;

Sys.time = 0:.5:max_time; 
Sys.ts=1; % sampling time for controller
Sys.L=10;   % horizon

% can move either up or left or right
Sys.u_ub = [1; 1];  % upper bound on u 
Sys.u_lb = [-1; 0]; % lower bound on u
% Sys.u_delta = [1; 1]; % not really sure what this does

Sys.Wref = [Sys.time*0.; Sys.time*0.];

Sys.stl_list = {};

% goal, make sure we are in our lane and past the intersection
%Sys.stl_list(end+1) = {'alw (ev (x1(t) < -1 and x2(t) < 1 and x2(t) > 0))'};
Sys.stl_list(end+1) = {'alw (ev (x2(t) <  1))'};
Sys.stl_list(end+1) = {'and alw (ev (x2(t) >  0))'};
Sys.stl_list(end+1) = {'and alw (ev (x1(t) < -1))'};

% safety, make sure we are always in our lane or in the intersection
Sys.stl_list(end+1) = {'and alw ((x1(t) > 0 and x1(t) < 1) or (x2(t) > 0 and x2(t) < 1))'};

% make sure we don't hit another car
% Sys.stl_list(end+1) = {'alw (abs(x1(t) - w1(t)) > 0.5 or abs(x2(t) - w2(t) > 0.5))'};

Sys.stl_list = {strjoin(Sys.stl_list)};

fprintf("getting controller...\n");
controller = get_controller(Sys)

fprintf("running...\n");
%Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
Sys= Sys.run_deterministic(controller);

%% Plot the path of the car over time
close;
figure;

h = animatedline('linewidth', 10);
axis([-3 3 -3 3]);
set(gca, 'fontsize', 25);

hold on;

% vertical road lines
plot([1 1], [-3 -1], [1 1], [1 3], 'LineWidth', 15, 'Color', 'k');
plot([-1 -1], [-3 -1], [-1 -1], [1 3], 'LineWidth', 15, 'Color', 'k');

% horizontal road lines
plot([-3 -1], [1 1], [1 3], [1 1], 'LineWidth', 15, 'Color', 'k');
plot([-3 -1], [-1 -1], [1 3], [-1 -1], 'LineWidth', 15, 'Color', 'k');

% vertical lane lines
plot([0 0], [-3 3], '--', 'LineWidth', 10, 'Color', 'k');

% horizonal lane lines
plot([-3 3], [0 0], '--', 'LineWidth', 10, 'Color', 'k');

for i=1:max_time+1
    hold on;
    title(['time=' num2str(Sys.time(i))], 'fontsize', 30);
    h2 = plot( Sys.system_data.X(1, i),  Sys.system_data.X(2, i), '^r-', 'MarkerSize', 25, 'MarkerFaceColor', 'r');
    addpoints(h, Sys.system_data.X(1, i), Sys.system_data.X(2, i));
    drawnow;
    
    pause(.05);
    delete(h2);
end
plot( Sys.system_data.X(1, i),  Sys.system_data.X(2, i), '^r-', 'MarkerSize', 25, 'MarkerFaceColor', 'r');
