function [vx, vy] = getFlow(I1, I2, sigma)

[M11, M12, M22, q1, q2] = getMq(I1, I2, sigma);
% v=-inv(M)*q，
% 注意都是点乘，点除，这样一次性处理全部的点
%vx vy的大小和原图大小一致
vx = (M22.*q1 - M12.*q2) ./ (M12.^2 - M11.*M22);
vy = (M11.*q2 - M12.*q1) ./ (M12.^2 - M11.*M22);

end

