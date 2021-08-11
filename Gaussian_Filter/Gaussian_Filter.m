clc
clear all
close all

I = imread('BellevueLot.jpg');

h = [1, 2, 1; 2, 4, 2; 1, 2, 1];
weight = 1/16;
h = weight * h;
R = I(:, :, 1);
G = I(:, :, 2);
B = I(:, :, 3);
Red_filter = imfilter(R, h);
Green_filter = imfilter(G, h);
Blue_filter = imfilter(B, h);

combined = cat(3, R, G, B);

figure;
imshow(Red_filter);

figure;
imshow(Green_filter);

figure;
imshow(Blue_filter);

figure;
imshow(combined)