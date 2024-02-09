% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 300;
dt          = 0.1;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
drone1 = Quadcopter(ax1);


% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drone1.update(t, dt);
    drone1.plot(t);
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

% Extract X, Y, and Z coordinates from the "path" matrix
X = drone1.path(1, :);
Y = drone1.path(2, :);
Z = drone1.path(3, :);

landing_speed_x = drone1.landing_speed(1, :);
landing_speed_y = drone1.landing_speed(2, :);
landing_speed_z = drone1.landing_speed(3, :);

input_1 = drone1.input_record(1, :);
input_2 = drone1.input_record(2, :);
input_3 = drone1.input_record(3, :);
input_4 = drone1.input_record(4, :);


% Create a 3D plot of the trajectory
figure(2);
plot3(X, Y, Z, 'b-', 'LineWidth', 2);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Quadcopter 3D Trajectory');

% Adding a legend
legend('Quadcopter Trajectory');

hold on;
% Define the coordinates and labels for the points
points = [
    0 0 0;
    0 0 5;
    0 2.5 5;
    0 0 7.5;
    0 -2.5 5;
    0 0 2.5;
    0 2.5 5;
    2.5 2.5 2.5;
    2.5 2.5 0
];
% Loop through the points and add labels
hold on;
for i = 1:size(points, 1)
    x = points(i, 1);
    y = points(i, 2);
    z = points(i, 3);
    label = "Point " + num2str(i-1);
    plot3(x, y, z, 'ro', 'MarkerSize', 8);
    text(x, y, z, label, 'FontSize', 12);
end
hold off;


%Ploting landing speed wrt time
velocity_magnitude = sqrt(landing_speed_x.^2 + landing_speed_y.^2 + landing_speed_z.^2);

% Calculate the average speed at each time step
num_time_steps = length(velocity_magnitude);

% Plot the average speed vs. time
figure(3);
plot(1:num_time_steps, velocity_magnitude);
xlabel('Time (ms)');
ylabel('Average Speed (m/s)');
title('Average Speed vs. Time');
grid on;

time = 1:length(input_1);
figure(4);
plot(time, input_1, 'r', 'LineWidth', 1.5);  % Input 1 in red
hold on;
plot(time, input_2, 'g', 'LineWidth', 1.5);  % Input 2 in green
plot(time, input_3, 'b', 'LineWidth', 1.5);  % Input 3 in blue
plot(time, input_4, 'm', 'LineWidth', 1.5);  % Input 4 in magenta

xlabel('Time (ms)');
ylabel('Input Gamma (rad^2/s^2)');
title('Input Gamma vs. Time');
legend('Input 1', 'Input 2', 'Input 3', 'Input 4');
grid on;
hold off;