function [ Jac, residual ] = deriveAnalytic( IRef, DRef, I, xi, K )
    % calculate analytic derivative

    % get shorthands (R, t)
    T = se3Exp(xi);
    R = T(1:3, 1:3);
    t = T(1:3,4);
    RKInv = R * K^-1;

    % ========= warp pixels into other image, save intermediate results ===============
    % these contain the x,y image coordinates of the respective
    % reference-pixel, transformed & projected into the new image.
    nImg = zeros(size(IRef))-10;
    mImg = zeros(size(IRef))-10;

    % these contain the 3d position of the transformed point
    % xp yp zp 保存3d点的空间位置
    xp = NaN(size(IRef));
    yp = NaN(size(IRef));
    zp = NaN(size(IRef));
    % size(IRef,2) 是IRef的第二维的size
    % IRef的第一维是高度，y。第二维是宽度，x。
    for n=1:size(IRef,2) % x
        for m=1:size(IRef,1) % y
            % point in reference image. note that the pixel-coordinates of the
            % point (1,1) are actually (0,0).
            % matlab 下标从1开始，DRef保存了深度
            % n-1;m-1 是图像点的位置，p乘上深度，现在的p是λx'.
            %是3d点经过projection和相机的intrinsic matrix
            % DRef(m,n)是高m，宽n的点，转成p就是换了维度，x，y
            p = [n-1;m-1;1] * DRef(m,n);

            % transform to image (unproject, rotate & translate)
            % KInv * p 是相机坐标系的3d点，然后再R，T
            pTrans = RKInv * p + t;

            % if point is valid (depth > 0), project and save result.
            if(pTrans(3) > 0 && DRef(m,n) > 0)
                % projected point (for interpolation of intensity and gradients)
                % 这里是像素坐标系，再projection到像素坐标系，
                pTransProj = K * pTrans;
                % nImg(m,n) 保存的是x,mImg(m,n)是y，
                nImg(m,n) = pTransProj(1) / pTransProj(3); 
                mImg(m,n) = pTransProj(2) / pTransProj(3);

                % warped 3d point, for calculation of Jacobian.
                % 这里计算下面算梯度要用的x，y，z，都是相机坐标系
                xp(m,n) = pTrans(1);
                yp(m,n) = pTrans(2);
                zp(m,n) = pTrans(3);
            end
        end
    end


    % ========= calculate actual derivative. ===============
    % 1.: calculate image derivatives, and interpolate at warped positions.
    % 根据公式，这里的x，y，z坐标都是从IRef，K等计算出来的
    % 所以不是准确的点，下面要插值
    dxI = NaN(size(I));
    dyI = NaN(size(I));
    %先算出梯度 再插值
    dyI(2:(end-1),:) = 0.5*(I(3:(end),:) - I(1:(end-2),:));
    dxI(:,2:(end-1)) = 0.5*(I(:,3:(end)) - I(:,1:(end-2)));
    %全部点的Ixfx，Iyfy，size(I,1) * size(I,2)×1，
    Ixfx = K(1,1) * reshape(interp2(dxI, nImg+1, mImg+1),size(I,1) * size(I,2),1);
    Iyfy = K(2,2) * reshape(interp2(dyI, nImg+1, mImg+1),size(I,1) * size(I,2),1);

    % 2.: get warped 3d points (x', y', z').
    % 最后把点展平
    xp = xp(:);
    yp = yp(:);
    zp = zp(:);

    % 3. implement gradient computed in Theory Ex. 1 (b)
    % jac是一个n×6的矩阵，n是点的个数
    Jac = zeros(size(I,1) * size(I,2),6);
    Jac(:,1) = Ixfx ./ zp;
    Jac(:,2) = Iyfy ./ zp;
    Jac(:,3) = - (Ixfx .* xp + Iyfy .* yp) ./ (zp .* zp);
    Jac(:,4) = - (Ixfx .* xp .* yp) ./ (zp .* zp) - Iyfy .* (1 + (yp ./ zp).^2);
    Jac(:,5) = + Ixfx .* (1 + (xp ./ zp).^2) + (Iyfy .* xp .* yp) ./ (zp .* zp);
    Jac(:,6) = (- Ixfx .* yp + Iyfy .* xp) ./ zp;

    % ========= plot residual image =========
    % ＋1 因为要插值，nImg，mImg都是从0开始，但是IRef从1开始
    residual = interp2(I, nImg+1, mImg+1) - IRef;
    imagesc(residual), axis image
    colormap gray
    set(gca, 'CLim', [-1,1])
    residual = residual(:);
end

