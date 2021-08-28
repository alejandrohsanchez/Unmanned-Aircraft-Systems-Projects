clc
clear all
close all

% Reference
% https://www.mathworks.com/matlabcentral/answers/151430-how-to-count-pixel-in-binary-image

he = imread('VP_Swale.jpg');
lab_he = rgb2hsv(he);

ab = lab_he(:, :, 2:3);
ab = im2single(ab);
nColors = 3;

pixel_labels = imsegkmeans(ab, nColors, 'NumAttempts', 3);

K = rgb2gray(he);
darkRockValue = 20;
noDarkRock = imextendedmax(K, darkRockValue);

figure;
imshow(he);

figure;
imshow(noDarkRock);

figure
imshow(pixel_labels, [])
title('Image Labeled by Cluster Index');

mask1 = pixel_labels == 1;
cluster1 = he .* uint8(mask1);

figure
imshow(cluster1)
title('Objects in Cluster 1');

mask2 = pixel_labels==2;
cluster2 = he .* uint8(mask2);

figure
imshow(cluster2)
title('Objects in Cluster 2');

mask3 = pixel_labels==3;
cluster3 = he .* uint8(mask3);

figure
imshow(cluster3)
title('Objects in Cluster 3');

totalPixels = numel(he);
sz = size(pixel_labels);
percentage = ((sz(1) * sz(2)) / totalPixels) * 100;
disp('The percentage of rocks in the image is:');
disp(percentage);
