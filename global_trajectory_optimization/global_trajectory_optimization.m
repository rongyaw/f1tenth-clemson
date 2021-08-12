%%  Global trajectory optimization script
% Author: Rongyao Wang
% Institution: Clemson University Mechanical Engineering
clear
clc
close all
dbstop if error
warning off

%% Load the original trajectory file
dirname = '/home/rongyaw/f1tenth_ws/src/f1tenth_simulator/f1tenth-clemson/global_trajectory_optimization/100_percent_minimum_distance_path';
raw_traj = load(strcat(dirname, '/path_curvature_100_lap_4.csv'),'r');
x = raw_traj(:,1);
y = raw_traj(:,2);
yaw = raw_traj(:,3);

[x, y, yaw] = repeat_data_filtering(x, y, yaw);
%%  Perform DP to calculate optimal velocity profile
% Set up basic parameter for limit friction navigation
mu = 0.723;
v_max = 7.0;
v_min = 0.0;

% Compute the velocity profile using dynamic programming method
[velocity, time] = velocity_profile(x, y, yaw, mu, v_max, v_min, (v_max - v_min)*100);

%%  Display the DP result and the speed profile
figure(1)
t = linspace(0, time, length(velocity));
[hAx, h1, h2] = plotyy(t, yaw, t, velocity);
xlabel('Time [s]')
ylabel(hAx(1), 'Yaw angle [rad]','Fontsize',13);
ylabel(hAx(2), 'Velocity [m/s]','Fontsize',13);
h1.LineStyle = '-';
h1.Color = 'b';
h1.LineWidth = 3;
h2.LineStyle = '-.';
h2.Color = 'r';
h2.LineWidth = 3;
legend('Yaw Angle','Velocity Profile');

figure(2)
scatter(x, y, 40, velocity,'filled');
hold on;
hc = colorbar;
hc.Label.String = 'Velocity [m/s]';
caxis([v_min, v_max]);
title('Velocity Profile of Trajectory');
xlabel('X [m]');
ylabel('Y [m]');

figure(3)
plot3(x, y, velocity);grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
%%  Write final trajecotory file
% new_traj = [x.',y.',yaw.',velocity];
% writematrix(new_traj,strcat(dirname,'initial_path.csv'));