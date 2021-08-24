% Notes:
% 理论上八点法的rank只能是8，留一个自由度，但是因为数据带噪声，所以可能rank是9
% - You will not see that rank(chi)==8 for real data, as the noise destroys
%   the nullspace. However, the rank will be 8 with perfect simulated data.
% - The algorithm is not robust if the correspondences are inaccurate. For
%   the given images, a set of predefined correspondences is given, for
%   which the algorithm should work. To hand-select points, it may be more
%   robust to use another image provided by matlab (see below).
% - If you hand-select points, do it in the same order for both images,
%   such that corresponding points have the same index

close all;
clear all;

%% obtain correspondences

% Method for selecting corresponding point pairs. (un)comment as needed
method = "predefined"; % use predefined correspondences
%method = "select"; % select correspondences by hand
%method = "simulate"; % simulate perfect correspondences (for debugging)

% Choose to use the images given with the exercise, or alternative ones
% provided by matlab. (un)comment as needed
% (no effect if method = "simulate")
%images = "given";
images = "matlab";

if strcmp(method, "simulate")
    % Random ground-truth pose
    R_gt = expm(hat(rand(3,1)*0.1));
    T_gt = rand(3,1);
    
    % camera intrinsics
    K1 = eye(3);
    K2 = eye(3);
    
    % generate random 3D points
    %
    nPoints = 12;
    % for corresponding pixel coordinates
    x1 = zeros(nPoints,1);
    y1 = zeros(nPoints,1);
    x2 = zeros(nPoints,1);
    y2 = zeros(nPoints,1);
    % for ground-truth 3D points
    X1_gt = zeros(3, nPoints);
    n = 0;
    while n < nPoints
        X1 = (rand(3, 1) - 0.5) * 20;
        X2 = R_gt*X1 + T_gt;
        % only accept if they are in front of both cameras
        if X1(3) > 0.1 && X2(3) > 0.1
            n = n+1;
            xy1 = K1 * X1 / X1(3);
            x1(n) = xy1(1);
            y1(n) = xy1(2);
            xy2 = K2 * X2 / X2(3);
            x2(n) = xy2(1);
            y2(n) = xy2(2);
            X1_gt(:, n) = X1;
        end
    end
    
 % use real images
else
    if strcmp(images, "matlab")
        imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');
        images = imageDatastore(imageDir);
        I1 = readimage(images, 1);
        I2 = readimage(images, 2);
        load upToScaleReconstructionCameraParameters.mat
        image1 = undistortImage(I1, cameraParams);
        image2 = undistortImage(I2, cameraParams);
        K1 = cameraParams.IntrinsicMatrix';
        K2 = K1;
        if strcmp(method, "predefined")
            [x1,y1,x2,y2] = getpoints_predefined_matlabimage();
        end
    elseif strcmp(images, "given")

        % Read input images:
        image1 = double(imread('batinria0.tif'));
        image2 = double(imread('batinria1.tif'));


        % Intrinsic camera parameter matrices
        K1 = [844.310547 0 243.413315; 0 1202.508301 281.529236; 0 0 1];
        K2 = [852.721008 0 252.021805; 0 1215.657349 288.587189; 0 0 1];
        
        if strcmp(method, "predefined")
            [x1,y1,x2,y2] = getpoints_predefined_givenimage();
        end
    end

    % choose corresponding points by hand
    if strcmp(method, "select")
        nPoints = 12; %or 8
        [x1,y1,x2,y2] = getpoints(image1,image2,nPoints);

    % use predefined points
    elseif strcmp(method, "predefined")
        nPoints = length(x1);
        % Show the selected points
        imshow(uint8(image1))
        hold on
        plot(x1, y1, 'r+')
        hold off
        figure
        imshow(uint8(image2))
        hold on
        plot(x2, y2, 'r+')
        hold off
    end
end
    

%% Pose and structure reconstruction

% Transform image coordinates with inverse camera matrices:
% normalized coordinate
xy1 = K1 \ [x1'; y1'; ones(1,nPoints)];
xy2 = K2 \ [x2'; y2'; ones(1,nPoints)];
x1 = xy1(1,:);
y1 = xy1(2,:);
x2 = xy2(1,:);
y2 = xy2(2,:);

% Compute constraint matrix chi:
chi = zeros(nPoints,9);
for i = 1:nPoints
    chi(i,:) = kron([x1(i) y1(i) 1],[x2(i) y2(i) 1]);
end

rank_chi = rank(chi)

% Find minimizer for |chi*E|:
[UChi,DChi,VChi] = svd(chi);

% Unstacked ninth column of V:
% 最后一列是特征值最小的vector
E = reshape(VChi(:,9),3,3);

% SVD of E
% U V in SO(3),所以det＞0
[U,D,V] = svd(E);
if det(U) < 0
    U = -U;
end
if det(V) < 0
    V = -V;
end

% Project E onto essential space (replace eigenvalues):
D(1,1) = 1;
D(2,2) = 1;
D(3,3) = 0;

% Final essential matrix:
E = U * D * V';

% Recover R and T from the essential matrix E:
% (Compare Slides)
Rz1 = [0 1 0; -1 0 0; 0 0 1]';
Rz2 = [0 -1 0; 1 0 0; 0 0 1]';
R1 = U * Rz1' * V';
R2 = U * Rz2' * V';
T_hat1 = U * Rz1 * D * U';
T_hat2 = U * Rz2 * D * U';

% Translation belonging to T_hat
T1 = [ -T_hat1(2,3); T_hat1(1,3); -T_hat1(1,2) ];
T2 = [ -T_hat2(2,3); T_hat2(1,3); -T_hat2(1,2) ];


% Compute scene reconstruction and correct combination of R and T:
n_success = 0;
[success, X1, ~] = reconstruction(R1,T1,x1,y1,x2,y2,nPoints);
if success
    n_success = n_success + 1;
    R_result = R1;
    T_result = T1;
    X1_result = X1;
end
[success, X1, ~] = reconstruction(R1,T2,x1,y1,x2,y2,nPoints);
if success
    n_success = n_success + 1;
    R_result = R1;
    T_result = T2;
    X1_result = X1;
end
[success, X1, ~] = reconstruction(R2,T1,x1,y1,x2,y2,nPoints);
if success
    n_success = n_success + 1;
    R_result = R2;
    T_result = T1;
    X1_result = X1;
end
[success, X1, ~] = reconstruction(R2,T2,x1,y1,x2,y2,nPoints);
if success
    n_success = n_success + 1;
    R_result = R2;
    T_result = T2;
    X1_result = X1;
end

switch n_success
    case 1
        disp("Success! Found one valid solution.")
    case 0
        disp("No valid solution found.")
    otherwise
        disp("Multiple valid solutions found.")
end

% for simulated correspondences, compare result to ground truth
if strcmp(method, "simulate")
    scale_factor = norm(T_gt) / norm(T_result);
    T_scaled = T_result * scale_factor;
    X1_scaled = X1_result * scale_factor;
    error_pose = norm(logm([R_result, T_scaled; 0,0,0,1]^-1 * [R_gt, T_gt; 0,0,0,1]))
    error_points = norm(X1_gt - X1_scaled)
end
    

%% function for structure reconstruction

% success specifies whether all depth values are positive
function [success, X1, x2] = reconstruction(R,T,x1,y1,x2,y2,nPoints)

% Structure reconstruction matrix M:
% 构造M矩阵，为了求深度
M = zeros(3*nPoints, nPoints + 1);
for i = 1:nPoints
   x2_hat = hat([x2(i) y2(i) 1]);
   
   M(3*i-2 : 3*i, i) = x2_hat*R*[x1(i); y1(i); 1];
   M(3*i-2 : 3*i, nPoints+1) = x2_hat*T;
end

% Get depth values (eigenvector to the smallest eigenvalue of M'M):
[V,D] = eig(M' * M);
%这里也是取特征值最小的列，但是在第一列
lambda1 = V(1:nPoints, 1);
gamma  = V(nPoints + 1, 1);

% Gamma has to be positive. If it is negative, use (-lambda1, -gamma), as
% it also is a solution for the system solved by (lambda1, gamma)
if gamma < 0
    gamma = -gamma;
    lambda1 = -lambda1;
end

% make lambda1 have the same scale as T
% 相当于T的系数是1
% 现在的T和lambda1可以按照比例进行缩放
lambda1 = lambda1 / gamma;

% generate 3D points,
% X1是3×n的矩阵，lambda1是n×1，
% repmat(lambda1', 3, 1)是把lambda1'在横向重复3次，变成3×n，
% 和 [x1; y1; ones(1, nPoints)] 点乘，变成3D点
X1 = [x1; y1; ones(1, nPoints)] .* repmat(lambda1', 3, 1);
%repmat(T, 1, nPoints)是把T重复1行，nPoints列，变成3×n,
X2 = R * X1 + repmat(T, 1, nPoints);
%lambda2是X2的深度
lambda2 = X2(3,:);

% Determine if we have the correct combination of R and T:
% 如果重建成功，那么所有的depth都是正数，不可能depth是负数，在camera后面
n_positive_depth1 = sum(lambda1 >= 0)
n_positive_depth2 = sum(lambda2 >= 0)
if n_positive_depth1==nPoints && n_positive_depth2==nPoints
    
    % show results
    R
    T
    X1
    
    figure
    
    plot3(X1(1,:), X1(2,:), X1(3,:), 'b+')
    
    hold on
    
    plotCamera('Location',[0 0 0],'Orientation',eye(3),'Opacity',0, 'Size', 0.2, 'Color', [1 0 0]) % red
    plotCamera('Location', -R'*T,'Orientation',R,'Opacity',0, 'Size', 0.2, 'Color', [0 1 0]) % green
    
    axis equal
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    hold off
    
    success = true;
else
    success = false;
end

end


%% Helper functions

% ================
% Hat-function
function A = hat(v)
    A = [0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0];
end



% ================
% function getpoints
function [x1,y1,x2,y2] = getpoints(image1,image2,nPoints)

x1 = zeros(nPoints,1);
y1 = zeros(nPoints,1);
x2 = zeros(nPoints,1);
y2 = zeros(nPoints,1);

% Click points in image1:
% Can be done without for-loop: ginput(nPoints)
figure; imshow(uint8(image1));
hold on;
for i = 1:nPoints
    [x,y] = ginput(1);
    x1(i) = double(x);
    y1(i) = double(y);
    plot(x, y, 'r+');
end
hold off;


% Click points in image2:
figure; imshow(uint8(image2));
hold on;
for i = 1:nPoints
    [x,y] = ginput(1);
    x2(i) = double(x);
    y2(i) = double(y);
    plot(x, y, 'r+');
end
hold off;

end



% ================
% predefined points for the given images
function [x1,y1,x2,y2] = getpoints_predefined_givenimage()

x1 = [
   10.0000
   92.0000
    8.0000
   92.0000
  289.0000
  354.0000
  289.0000
  353.0000
   69.0000
  294.0000
   44.0000
  336.0000
  ];

y1 = [ 
  232.0000
  230.0000
  334.0000
  333.0000
  230.0000
  278.0000
  340.0000
  332.0000
   90.0000
  149.0000
  475.0000
  433.0000
    ];
 
x2 = [
  123.0000
  203.0000
  123.0000
  202.0000
  397.0000
  472.0000
  398.0000
  472.0000
  182.0000
  401.0000
  148.0000
  447.0000
    ];

y2 = [ 
  239.0000
  237.0000
  338.0000
  338.0000
  236.0000
  286.0000
  348.0000
  341.0000
   99.0000
  153.0000
  471.0000
  445.0000
    ];

end

% ================
% predefined points for the images provided by matlab
function [x1,y1,x2,y2] = getpoints_predefined_matlabimage()

x1 = 1.0e+03 * [
    0.1692
    0.5515
    0.4525
    0.4165
    1.4241
    1.4345
    1.6699
    1.6744
    1.3011
    1.1542
    0.9938
    1.1647
];

y1 = [
  887.1881
  932.1662
  467.3931
  110.5673
  146.5498
  392.4297
  392.4297
  677.2906
  834.7138
  770.2452
  554.3507
  434.4092
];

x2 = 1.0e+03 * [
    0.2651
    0.6249
    0.5680
    0.5365
    1.5440
    1.5545
    1.8154
    1.8019
    1.2951
    1.1452
    0.9953
    1.1647
];

y2 = [
  869.1969
  921.6713
  473.3902
  136.0549
  143.5512
  395.4283
  390.9305
  690.7840
  849.7064
  782.2394
  564.8455
  449.4019
];

end
