function [vx, vy] = getSparseFlow(I1, I2, sigma, kappa, theta)

% for simplicity we compute it everywhere, then set to zero at non-corners

% compute flow
[M11, M12, M22, q1, q2] = getMq(I1, I2, sigma);
% 注意都是点乘和点除，一次性处理全部的点 
% v=-inv(M)*q，
%vx vy的大小和原图大小一致
vx = (M22.*q1 - M12.*q2) ./ (M12.^2 - M11.*M22);
vy = (M11.*q2 - M12.*q1) ./ (M12.^2 - M11.*M22);

% compute Harris corner score using det and trace
detM = M11 .* M22 - M12.^2;
traceM = M11 + M22;
score = detM - kappa * traceM.^2;

% padded image for easier non-max suppression check
score_pad = -inf * ones(size(I1) + 2);
score_pad(2:end-1, 2:end-1) = score;

% find corner indices
corner_idx = ( score > theta ...
             & score > score_pad(1:end-2, 2:end-1) ...
             & score > score_pad(3:end  , 2:end-1) ...
             & score > score_pad(2:end-1, 1:end-2) ...
             & score > score_pad(2:end-1, 3:end));

% 把非特征点的速度设置为0，只保留特征点的速度         
vx(~corner_idx) = 0;
vy(~corner_idx) = 0;

end

