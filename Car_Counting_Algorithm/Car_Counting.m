clc
clear all
close all

% Reference
% https://www.mathworks.com/help/images/detecting-cars-in-a-video-of-traffic.html

I = imread('Parking_Lot.jpg');

K = rgb2gray(I);
darkCarValue = 50;
noDarkCar = imextendedmax(K, darkCarValue);

figure;
imshow(noDarkCar);