function [M11, M12, M22, q1, q2] = getMq(I1, I2, sigma)
% Compute structure tensor

% spatial gradient using central differences

% use I1 here since we compute flow for I1
% Ix [2:end end] 2,3,4...end, end
% [1 1:end-1] 1,1,2..end-2,end-1
% 2Ix= 2-1,3-1,4-2....end-(end-2),end-(end-1)
% 中间每个Ix都是两边决定的，但是两头是例外，因为只有一边
% 同理Iy
Ix = 0.5*(I1(:,[2:end end]) - I1(:,[1 1:end-1]));
Iy = 0.5*(I1([2:end end],:) - I1([1 1:end-1],:));

% temporal gradient with forward differences
% 计算l在时间上的微分
It = I2 - I1;

% gaussian kernel
% sigma是标准差，k是卷积核大小，
%如果sigma=1，k=5，5×5是卷积核大小
k = ceil(4*sigma+1);
G = fspecial('gaussian', k, sigma);

% 用了weights的M在window里用了权重，卷积是small window的结果
%注意都是点乘
M11 = conv2(Ix .* Ix, G, 'same');
M12 = conv2(Ix .* Iy, G, 'same');
M22 = conv2(Iy .* Iy, G, 'same');

q1 = conv2(Ix .* It, G, 'same');
q2 = conv2(Iy .* It, G, 'same');

end
