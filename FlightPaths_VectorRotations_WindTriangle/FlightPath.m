clc
clear all
close all

%---- Part 1 ----%
% Creating Matlab script that loads Drone_Data.mat, plots flight path, and
% plots waypoints within the same figure as the flight path
load('Drone_Data.mat'); % Loading Drone_Flight and Drone_Waypoints from Drone_Data.mat
x1 = Drone_Flight(2,:); % Position East
y1 = Drone_Flight(1,:); % Position North

x2 = Drone_Waypoints(:,2); % Waypoint x coordinates
y2 = Drone_Waypoints(:,1); % Waypoint y coordinates

figure
plot(x1,y1);
axis equal
hold on
plot(x2,y2,'r*');
hold off
legend('Flight Path', 'Waypoints');
title('Drone Flight Path')