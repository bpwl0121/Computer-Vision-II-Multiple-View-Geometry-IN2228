function [ Id, Dd, Kd ] = downscale( I, D, K, level )
    % 这个函数的作用是下采样，把图片缩小
    % downscale intensity image, depth map and camera intrinsics (recursively)

    if(level <= 1)
        % coarsest pyramid level
        Id = I;
        Dd = D;
        Kd = K;
        return;
    end

    % downscale camera intrinsics
    % this is because we interpolate in such a way, that 
    % the image is discretized at the exact pixel-values (e.g. 3,7), and
    % not at the center of each pixel (e.g. 3.5, 7.5).
    % intrinsics matrix 根据练习进行缩小，f和c
    Kd = [K(1,1)/2, 0, (K(1,3)+0.5)/2-0.5;
            0, K(2,2)/2, (K(2,3)+0.5)/2-0.5;
            0, 0, 1];

    % downscale intensity image
    % 像素的下采样，每个L+1层的像素都是原来L层的周围的4个像素的平均值
    % I(0+(1:2:end), 0+(1:2:end)是原来矩阵奇数位置的数值
    %  I=magic(10) I(0+(1:2:end), 0+(1:2:end)) 试试这个就懂了
    Id = (I(0+(1:2:end), 0+(1:2:end)) + ...
        I(1+(1:2:end), 0+(1:2:end)) + ...
        I(0+(1:2:end), 1+(1:2:end)) + ...
        I(1+(1:2:end), 1+(1:2:end)))*0.25;

    % downscale depth map
    % 有效像素，是depth不是0的L层的周围的像素个数，最多4个，最少0个
    DdCountValid = (sign(D(0+(1:2:end), 0+(1:2:end))) + ...
                sign(D(1+(1:2:end), 0+(1:2:end))) + ...
                sign(D(0+(1:2:end), 1+(1:2:end))) + ...
                sign(D(1+(1:2:end), 1+(1:2:end))));
    %Dd是L+1层的像素depth是原来L层的周围的4个像素depth的平均值
    Dd = (D(0+(1:2:end), 0+(1:2:end)) + ...
        D(1+(1:2:end), 0+(1:2:end)) + ...
        D(0+(1:2:end), 1+(1:2:end)) + ...
        D(1+(1:2:end), 1+(1:2:end))) ./ DdCountValid;
    Dd(isnan(Dd)) = 0;

    % recursively downscale on next pyramid level
    % 递归下采样
    [Id, Dd, Kd] = downscale( Id, Dd, Kd, level - 1);    
end