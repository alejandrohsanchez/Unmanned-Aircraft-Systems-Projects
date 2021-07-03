clc
clear all
close all

%---- Part 3 ----%
% This part of the lab will simulate an aircraft moving in a 3D space and
% determine its final position after 2 seconds, given its respective roll,
% pitch and yaw. Additionally, it will simulate a windspeed that will alter
% the aircrafts groundspeed and show its path on a 3D graph.

strt = [50, 100, -20]; % Starting position
v1 = [8, 0, -1];
dt = 2;

R = ro(0,6,0,'y');
v1 = v1*R.';
R = ro(0,0,120,'z');
v1 = v1*R.';

end1 = strt + (v1*dt) % anticipated flight path

x = [strt(:,1),end1(:,1)];
y = [strt(:,2),end1(:,2)];
z = [strt(:,3),end1(:,3)];
plot3(x,y,z,'k-');
grid on
axis equal
xlabel('East')
ylabel('North')
zlabel('Down (Absolute)')
title('Wind Triangle')
hold on

vw = [1, 1, -1]; % wind vector (this can be changed to be custom)
R = ro(0,6,0,'y');
vwb = vw*R.';
R = ro(0,0,120,'z');
vwb = vw*R.'; % wind vector expressed in body frame

vg = end1 + vw; % new ending of aircraft with wind effects
va = vg - vwb;


x = [strt(:,1),vg(:,1)];
y = [strt(:,2),vg(:,2)];
z = [strt(:,3),vg(:,3)];
plot3(x,y,z,'b-');
hold on

vg = strt + vw;
end2 = vg - vwb;
x = [vg(:,1), end2(:,1)];
y = [vg(:,2), end2(:,2)];
z = [vg(:,3), end2(:,3)];
plot3(x,y,z,'r-');
hold off

aspeed = sqrt(sum(va.^2)); % airspeed
alpha = atan(va(:,3)/va(:,1)); % angle of attack of wind
beta = asind(va(:,2)/aspeed); % side-slip angle

legend('Airspeed & Heading','Actual Course/Groundspeed','Windspeed/Direction','Location','northwest');

function R = ro(phi, theta, psi, axis)
    if (axis == 'z') % yaw
        R = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1];
        
    elseif (axis == 'y') % pitch
        R = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)];
        
    elseif (axis == 'x') % roll
        R = [1, 0, 0; 0, cos(psi), sin(psi); 0, -sin(psi), cos(psi)];
    end
end