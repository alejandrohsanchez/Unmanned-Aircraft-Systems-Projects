clear;
clc;
close all;


load Part3_data


t = IMU.time;
[C,ia,ic] = unique(t);
ax = IMU.gFx;
ax = ax(ia);
ay = IMU.gFy;
ay = ay(ia);
az = IMU.gFz;
az = az(ia);
gx = rad2deg(IMU.wx);
gx = gx(ia);
gy = -rad2deg(IMU.wy);
gy = gy(ia);
gz = rad2deg(IMU.wz);
gz = gz(ia);
dt = 0.0024;

samples = length(C);

a_phi = zeros(samples,1);
a_theta = zeros(samples,1);
g_phi = zeros(samples,1);
g_theta = zeros(samples,1);
phi = zeros(samples,1);
theta = zeros(samples,1);

fc = 1/2;
a = 1/(2*pi* fc);% fc is a user parameter

for i=2:samples
    dt(i) = C(i)-C(i-1);
    % dt ~ 0.003 - 0.004
      
    %Acc estimate  
    a_phi(i) = atan2(ay(i),az(i))*180/pi;
    a_theta(i) = atan2(ax(i),az(i))*180/pi;
    
    %Gyro estimate
    g_phi(i) = g_phi(i-1)+dt(i)*gx(i);
    g_theta(i) = g_theta(i-1)+dt(i)*gy(i);
    
    
    %Complementary Filter Est
    % k is k_coeff
    k = a/(a+dt(i));
    
    % Implementing complementary filter for roll and pitch
    phi(i) = (k * g_phi(i)) + ((1 - k) * a_phi(i));
    theta(i) = (k * g_theta(i)) + ((1 - k) * a_theta(i));
    
    

end

figure;
%Title and label
plot(C,phi,'-b',C,a_phi,'-r',C,g_phi,'-k');
title('Filter Estimate Plot for Roll')
legend('Complementary Estimate Filter','Accelerometer Estimate', 'Gyroscope Estimate');

figure;
%Title and label
plot(C,theta,'-b',C,a_theta,'-r',C,g_theta,'-k');
title('Filter Estimate Plot for Pitch')
legend('Complementary Estimate Filter','Accelerometer Estimate', 'Gyroscope Estimate')
