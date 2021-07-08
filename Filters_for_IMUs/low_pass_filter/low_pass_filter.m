clear;
clc;
close all;

%Time parameters
dt = 0.05;
t = 0:dt:20;

%Generate an input signal
y_clean = 3*sin(t*0.05*pi);
y_noise = 0.1*randn(1,length(t));
y_total = y_clean+y_noise;

%Filter parameters
tau = 1/5;
alpha = dt/(tau+dt);

g = zeros(1,length(t));
for i = 2:length(t)
    g(i) = ((1 - alpha) * g(i - 1)) + (alpha * y_total(i));
end

%Plot the data;
figure()
plot(t,y_total)
hold on
plot(t,g)

title('Low Pass Filter')
xlabel('time (s)')
ylabel('data')
legend('Data with Noise','Data without noise')
subtitle('$$\tau = \frac{1}{5}$$','Interpreter','latex')