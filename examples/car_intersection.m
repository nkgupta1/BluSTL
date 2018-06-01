clear;
clc;
close;
% Example of car moving forward in a grid, and trying to make a left turn
% on a yield. There is oncoming traffic. We kinda cheat by allowing the car
% to move in a direction without explictly turning. This is a
% simplification until integrating piecewise linear works.
% 
SC = car_intersection_class(); 

% have to increase robustness to satisfy STL
SC.min_rob = 0.5;

% x, y
SC.x0= [0.5 ; -3];

max_time = 20;
ts = 0.5;

SC.time = 0:ts:max_time; 
SC.ts=1; % sampling time for controller
SC.L=10;   % horizon

% can move either up or left
SC.u_ub = [0; 1];  % upper bound on u 
SC.u_lb = [-1; 0]; % lower bound on u
Sys.u_delta = [.1; .1];

% first  row is x coordinate of center vehicle
% second row is y coordinate of center vehicle
SC.Wref = [SC.time*0.; SC.time*0.];

% default position of vehicle is off screen
SC.Wref(:,:) = -10;

% stationary test
SC.Wref(1,:) = -0.5;
SC.Wref(2,1:10/ts) = linspace(3,-3,10/ts);

SC.stl_list = {};

% goal, make sure we are in our lane and past the intersection
SC.stl_list(end+1) = {'ev (alw (x1(t) < -2))'};

% safety, make sure we are always in our lane
SC.stl_list(end+1) = {'and alw ((x1(t) > 0 and x1(t) < 1) or (x2(t) > 0 and x2(t) < 1))'};

% make sure we don't hit another car
% the other vehicle is 0.5*2 units wide of 2*2 units long
% build in a buffer of 0.1
x_size = 0.3;
y_size = 0.6;
SC.stl_list(end+1) = {'and alw (abs(x1(t) - w1(t)) > 0.4 or abs(x2(t) - w2(t)) > 0.7)'};

SC.stl_list = {strjoin(SC.stl_list)};

fprintf("getting controller...\n");
controller = get_controller(SC)

fprintf("running...\n");
%Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
SC= SC.run_deterministic(controller);

%% Plot the path of the car over time
close;
figure('pos', [0 0 600 600]);

h = animatedline('linewidth', 10);
axis([-3 3 -3 3]);
set(gca, 'fontsize', 25);
xlabel('x1');
ylabel('x2');

hold on;

% vertical road lines
plot([1 1], [-3 -1], [1 1], [1 3], 'LineWidth', 10, 'Color', 'k');
plot([-1 -1], [-3 -1], [-1 -1], [1 3], 'LineWidth', 10, 'Color', 'k');

% horizontal road lines
plot([-3 -1], [1 1], [1 3], [1 1], 'LineWidth', 10, 'Color', 'k');
plot([-3 -1], [-1 -1], [1 3], [-1 -1], 'LineWidth', 10, 'Color', 'k');

% vertical lane lines
plot([0 0], [-3 3], '--', 'LineWidth', 5, 'Color', 'k');

% horizonal lane lines
plot([-3 3], [0 0], '--', 'LineWidth', 5, 'Color', 'k');

folder = 'images/intersection2/';

for i=1:size(SC.system_data.W,2)
    hold on;
    title(['time=' num2str(SC.time(i))], 'fontsize', 20);
    
    x_coor = SC.system_data.W(1, i) - x_size;
    y_coor = SC.system_data.W(2, i) - y_size;
    width  = x_size*2;
    height = y_size*2;
        
    r = rectangle('Position', [x_coor y_coor width height], 'LineWidth', 3, 'FaceColor', [0 0 0 0.5]);
    
    h2 = plot( SC.system_data.X(1, i),  SC.system_data.X(2, i), '^r-', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
    addpoints(h, SC.system_data.X(1, i), SC.system_data.X(2, i));
    drawnow;
    
    %pause(.1);
    
    saveas(gcf, [folder  sprintf('%03d', i) '.png']);
    
    delete(h2);
    delete(r);
    
end
x_coor = SC.system_data.W(1, i) - x_size;
y_coor = SC.system_data.W(2, i) - y_size;
width  = x_size*2;
height = y_size*2;

r = rectangle('Position', [x_coor y_coor width height], 'LineWidth', 3, 'FaceColor', [0 0 0 0.5]);
plot( SC.system_data.X(1, i),  SC.system_data.X(2, i), '^r-', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

saveas(gcf, [folder  sprintf('%03d', i) '.png']);