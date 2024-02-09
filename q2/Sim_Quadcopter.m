% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 10;
dt          = 0.1;
TIME_SCALE  = 1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


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
    drone1.update(t,dt);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

position = drone1.pos_record(3,:);
velocity = drone1.pos_dot_record(3,:);
theta = drone1.theta_record(3,:);
omega = drone1.omega_record(3,:);

state_record = [position; velocity; theta; omega];

time = 0:length(position)-1;

figure(2);
state_record(1) = plot(time, position, 'r', 'LineWidth', 1.5);
hold on;
state_record(2) = plot(time, velocity, 'g', 'LineWidth', 1.5);
state_record(3) = plot(time, theta, 'b', 'LineWidth', 1.5);
state_record(4) = plot(time, omega, 'm', 'LineWidth', 1.5);

xlabel('Time (ms)');
ylabel('State values');
title('Quadcopter state values (z-axis) vs. Time');

legend( 'Position (m)', 'Velocity (m/s)', 'Theta (rad)', 'Omega (rad/s)');
grid on;
hold off;

X = drone1.path(1, :);
Y = drone1.path(2, :);
Z = drone1.path(3, :);

figure(3);
plot3(X, Y, Z, 'r-', 'LineWidth', 2);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Quadcopter 3D Trajectory');

% Adding a legend
legend('Quadcopter Trajectory');