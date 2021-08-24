function [ err ] = calcErr( IRef, DRef, I, xi, K )

    % IRef是第一张图
    % I是第二张图
    % xi是李代数的刚体变换，把xi转成R和T
    % 首先把IRef的点还原成λx'，通过DRef，就是Z，
    % 再通过xi，K重建I图像，就是把IRef的像素坐标通过变换得到I的像素坐标
    % 例如1，2变成了4.5,7.3,意思是在IRef坐标的1，2的像素会变成I图像4.5,7.3像素
    % 通过线性插值得到通过变换得到的I的图像
    % 最后重建的I和给的I做残差
    %最后把err展平
    % calculate residuals

    % get shorthands (R, t)
    T = se3Exp(xi);
    R = T(1:3, 1:3);
    t = T(1:3,4);

    RKInv = R * K^-1;

    % these contain the x,y image coordinates of the respective
    % reference-pixel, transformed & projected into the new image.
    % set to -10 initially, as this will give NaN on interpolation later.
    nImg = zeros(size(IRef))-10;
    mImg = zeros(size(IRef))-10;

    % for all pixels
    % size(IRef) = 480   640, n是640，m是480
    %https://blog.csdn.net/qq_39790756/article/details/77010267
    for n=1:size(IRef,2)
        for m=1:size(IRef,1)
            % point in reference image. note that the pixel-coordinates of the
            % point (1,1) are actually (0,0).
            % n是行，m是列，原来的像素点乘深度，是λx',
            % 点坐标x是列，y是行，所以有反转，图像从0，0开始
            p = [n-1;m-1;1] * DRef(m,n);

            % transform to image (unproject, rotate & translate)
            % KInv * p是点p在相机坐标系的位置，再rotation和translation
            % 得到新的坐标点再相机坐标系上
            pTrans = K * (RKInv * p + t);

            % if point is valid (depth > 0), project and save result.
            % 新的深度和原来的深度都要＞0
            if(pTrans(3) > 0 && DRef(m,n) > 0)
                % +1是把原来以0，0为起点改为1，1为起点，matlab
                % 得到的新的点pTrans，
                %nImg(m,n) 保存了在原来n行m列的新点对应点的行
                %mImg(m,n) 保存了在原来n行m列的新点对应点的列
                nImg(m,n) = pTrans(1) / pTrans(3) + 1;
                mImg(m,n) = pTrans(2) / pTrans(3) + 1;
            end
        end
    end

    % calculate actual residual (as matrix).
    % 线性插值得到新图的点，
    err = interp2(I, nImg, mImg) - IRef;
    

    % plot residual image
    imagesc(err), axis image
    colormap gray;
    set(gca, 'CLim', [-1,1]);

    err = err(:);

end

