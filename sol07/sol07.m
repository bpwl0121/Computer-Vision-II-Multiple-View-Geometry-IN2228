function mvg_exerciseSheet_07_part2()

% Read images:
image1 = double(imread('batinria0.pgm'));
image2 = double(imread('batinria1.pgm'));
[w,h] = size(image2);

% Left camera parameters:
K1 = [844.310547 0 243.413315; 0 1202.508301 281.529236; 0 0 1];
R1 = [0.655133, 0.031153, 0.754871;0.003613, 0.999009, -0.044364;-0.755505, 0.031792, 0.654371];
T1 = [-793.848328; 269.264465; -744.572876];

% Right camera parameters:
K2 = [852.721008 0 252.021805; 0 1215.657349 288.587189; 0 0 1];
R2 = [0.739514, 0.034059, 0.672279;-0.006453, 0.999032, -0.043515;-0.673111, 0.027841, 0.739017];
T2 = [-631.052917; 270.192749; -935.050842];

% Compute the fundamental matrix:
g1 = [R1 T1; 0 0 0 1];
g2 = [R2 T2; 0 0 0 1];
g = inv(g2) * g1;
%g = g2* inv(g1) ;
T = g(1:3,4);
R = g(1:3,1:3);
% 两张图片的K不同
F = inv(K2)' * hat(T) * R * inv(K1)


% Click x1 in the first image:
figure; imshow(uint8(image1));
hold on;
% [x1,y1] 是选的点
[x1,y1] = ginput(1);
plot(x1,y1,'r+');
hold off;

% Compute epipolar line for x1:
l = F * [x1; y1; 1]

% Draw epipolar line in image2:
figure; imshow(uint8(image2));
hold on;
m = -l(1)/l(2);
b = -l(3)/l(2);
y1 = m * 1 + b;
y2 = m * w + b;
line([1 w],[y1 y2]);



%% Additional Part
% 用NCC求对应点
% Window size for ncc:

n = 3;

% Number of pixels inside the window
% window里pixel的数量
nb = (2*n+1)^2;

% Initialize ncc result:
maxncc = -10;
nx = 0;
ny = 0;

% Compute mean intensity value inside window of image1:
% 求图片1的window里的mean
mean1 = 0;
x1 = round(x1);
y1 = round(y1);
for j=-n:n
for i=-n:n
    mean1 = mean1 + image1(y1+j, x1+i);
end
end
%求得x1，y1窗口的mean
mean1 = mean1/nb;

% Compute NCC for all pixels on the epipolar line:
for x=n+1:w-n
    y = round(m * x + b);
    if (y < n+1) y = n+1; end
    if (y > h-n) y = h-n; end

    % Compute mean intensity value inside window of image2:
    mean2 = 0;
    for j=-n:n
    for i=-n:n
        mean2 = mean2 + image2(y+j, x+i);
    end
    end
    
    mean2 = mean2/nb;

    % Compute integrals of NCC:
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    for j=-n:n
    for i=-n:n
        v1 = image1(y1+j, x1+i) - mean1;
        v2 = image2(y+j, x+i) - mean2;
        % sum1是分子
        sum1 = sum1 + v1 * v2;
        % sum2，sum3是分母的部分
        sum2 = sum2 + v1 * v1;
        sum3 = sum3 + v2 * v2;
    end
    end

    % Compute NCC:
    ncc = sum1 / sqrt(sum2 * sum3);
    if (ncc > maxncc) 
        maxncc = ncc;
        nx = x;
        ny = y;
    end
end
plot(nx,ny,'r+');
hold off;
end

function A = hat(v)
A = [0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0];
end
