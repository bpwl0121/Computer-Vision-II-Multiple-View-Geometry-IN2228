%% cleanup
clear
close all

%% load data and set parameters
I = imreadbw('img1.png');
%I = imreadbw('small.png');
sigma = 20;
kappa = 0.05;
theta = 1e-7;

%% compute corners and visualize 

[score, points] = getHarrisCorners(I, sigma, kappa, theta);

figure(2);
subplot(121);
drawPts(I, points);
axis image;

subplot(122);
imagesc(sign(score) .* abs(score).^(1/4));
colormap(gca, 'jet');
axis image;
colorbar;
