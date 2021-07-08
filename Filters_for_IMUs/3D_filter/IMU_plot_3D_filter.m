clear;
clc;
close all

load Part2_IMU_data

%Time parameters
dt = 0.3;
t = dt*(1:N);

%Filter parameters
tau = 1/3;
alpha = dt/(tau+dt);

acc_x_filt = zeros(N,1);
acc_y_filt = zeros(N,1);
acc_z_filt = zeros(N,1);

acc_x_filt(1) = acc_x(1);
acc_y_filt(1) = acc_y(1);
acc_z_filt(1) = acc_z(1);

for i = 2:N
    acc_x_filt(i) = ((1 - alpha) * acc_x_filt(i - 1)) + (alpha * acc_x(i));
    acc_y_filt(i) = ((1 - alpha) * acc_y_filt(i - 1)) + (alpha * acc_y(i));
    acc_z_filt(i) = ((1 - alpha) * acc_z_filt(i - 1)) + (alpha * acc_z(i));
end


% Plotting Data
figure
plot(t, acc_x)
hold on
plot(t, acc_x_filt)

title('Low Pass Filter for x axis (accelerometer)')
xlabel('time (s)')
ylabel('data')
legend('Data with Noise','Data without noise')
subtitle('$$\tau = \frac{1}{3}$$','Interpreter','latex')

figure
plot(t, acc_y)
hold on
plot(t, acc_y_filt)

title('Low Pass Filter for y axis (accelerometer)')
xlabel('time (s)')
ylabel('data')
legend('Data with Noise','Data without noise')
subtitle('$$\tau = \frac{1}{3}$$','Interpreter','latex')

figure
plot(t, acc_z)
hold on
plot(t, acc_z_filt)

title('Low Pass Filter for z axis (accelerometer)')
xlabel('time (s)')
ylabel('data')
legend('Data with Noise','Data without noise')
subtitle('$$\tau = \frac{1}{3}$$','Interpreter','latex')