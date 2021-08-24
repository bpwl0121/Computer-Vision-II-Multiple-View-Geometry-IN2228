function [M11, M12, M22] = getM(I, sigma)
% Compute structure tensor

% spatial gradient using central differences
% Ix [2:end end] 2,3,4...end, end
% [1 1:end-1] 1,1,2..end-2,end-1
% 2Ix= 2-1,3-1,4-2....end-(end-2),end-(end-1)
% 中间每个Ix都是两边决定的，但是两头是例外，因为只有一边
% 同理Iy
Ix = 0.5*(I(:,[2:end end]) - I(:,[1 1:end-1]));
Iy = 0.5*(I([2:end end],:) - I([1 1:end-1],:));

% Gaussian kernel
k = ceil(4*sigma+1);
G = fspecial('gaussian', k, sigma);

% 用了weights的，注意都是点乘
M11 = conv2(Ix .* Ix, G, 'same');
M12 = conv2(Ix .* Iy, G, 'same');
M22 = conv2(Iy .* Iy, G, 'same');

% Note: We do not need to compute M21 sparately, since M is symmetrical and
% therefore M21 == M12

end
