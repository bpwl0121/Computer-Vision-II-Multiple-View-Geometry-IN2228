function [score, points] = getHarrisCorners(I, sigma, kappa, theta)

[M11, M12, M22] = getM(I, sigma);

% compute score using det and trace
% 得到M矩阵算score，这个score大于theta，则是Harris corner 点，（没用Non-maximum suppression）
% 注意都是点乘,M11，M22，M12里面保存每个点的M信息，也就是说每个M都是480×640
detM = M11 .* M22 - M12.^2;
% 求trace
traceM = M11 + M22;
score = detM - kappa * traceM.^2;

% display score (for debugging)
%imagesc(sign(score) .* abs(score).^(1/4));
%colorbar;

% you can disable non-maximum suppression (for debugging)
% max_only设置为0，就没有non-max suppression 
max_only = 1;

% padded image for easier non-max suppression check
% 在原来scores周围pad一圈，为了Non-maximum suppression在边界更方便
score_pad = -inf * ones(size(I) + 2);
score_pad(2:end-1, 2:end-1) = score;

% find corners
% score的值要超过theta，因为在480×640的点里，有的M是0，或者很小，说明他的梯度不够显著
% 例如M等于0，是一个纯白的点，看不出运动，或者没有运动，
% 或者非角点，det（M)是0，一个特征值
% Non-maximum suppression,score的值还要比周围的值都要大，只检查了上下左右
%Non-maximum suppression 第一行，比左边大。第二行，比右边大。第三行，比上面大，第四行，比下面大
[y, x] = find( score > theta ...
             & ( ~max_only ...
               | ( score > score_pad(1:end-2, 2:end-1) ...
                 & score > score_pad(3:end  , 2:end-1) ...
                 & score > score_pad(2:end-1, 1:end-2) ...
                 & score > score_pad(2:end-1, 3:end))));

points = [x y];

end