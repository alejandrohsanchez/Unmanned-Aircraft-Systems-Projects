clc
clear all
close all


%---- Part 2 ----%
% This part will generate a function that generates a Rotation Matrix. It
% will test the function by rotating 3D vectrors and plotting them.
v1 = [1, 1, 1];
x = [0 v1(:,1)];
y = [0 v1(:,2)];
z = [0 v1(:,3)];

figure
plot3(x,y,z) % This is plotting the first vector, A
grid on
axis equal
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
legend('Vector A','Location','northwest')
title('Pre-Rotated Vector')


R = ro(90,0,0,'z'); % Rotating about the z axis 90 degrees
v2 = v1*R;

x2 = [0 v2(:,1)];
y2 = [0 v2(:,2)];
z2 = [0 v2(:,3)];

figure
plot3(x2,y2,z2);
grid on
axis equal
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
hold on

R = ro(0,90,0,'y'); % Rotating about the y axis 90 degrees
v2 = v1*R;

x2 = [0 v2(:,1)];
y2 = [0 v2(:,2)];
z2 = [0 v2(:,3)];

plot3(x2,y2,z2);
hold on

R = ro(0,45,0,'y'); % Rotating about the 45 axis 45 degrees
v2 = v1*R;

x2 = [0 v2(:,1)];
y2 = [0 v2(:,2)];
z2 = [0 v2(:,3)];

plot3(x2,y2,z2);
hold on

R = ro(0,0,180,'x'); % Rotating about the 45 axis 45 degrees
v2 = v1*R;

x2 = [0 v2(:,1)];
y2 = [0 v2(:,2)];
z2 = [0 v2(:,3)];

plot3(x2,y2,z2);
hold off
legend('Vector A 90 degrees about z axis','Vector A 90 degrees about y axis','Vector A 45 degrees about y axis','Vector A 180 degrees about x axis','Location','northwest')

% The following sub function generates a rotation matrix and returns it to
% the program above to calculate new vectors rotated about their respective
% axes
function R = ro(phi, theta, psi, axis)
    if (axis == 'z')
        R = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1];
        
    elseif (axis == 'y')
        R = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)];
        
    elseif (axis == 'x')
        R = [1, 0, 0; 0, cos(psi), sin(psi); 0, -sin(psi), cos(psi)];
    end
end