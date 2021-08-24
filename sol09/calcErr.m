function [ err ] = calcErr( IRef, DRef, I, xi, K )
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
    for n=1:size(IRef,2)
        for m=1:size(IRef,1)
            % point in reference image. note that the pixel-coordinates of the
            % point (1,1) are actually (0,0).
            p = [n-1;m-1;1] * DRef(m,n);

            % transform to image (unproject, rotate & translate)
            pTrans = K * (RKInv * p + t);

            % if point is valid (depth > 0), project and save result.
            if(pTrans(3) > 0 && DRef(m,n) > 0)
                nImg(m,n) = pTrans(1) / pTrans(3) + 1;
                mImg(m,n) = pTrans(2) / pTrans(3) + 1;
            end
        end
    end

    % calculate actual residual (as matrix).
    err = interp2(I, nImg, mImg) - IRef;

    % plot residual image
    imagesc(err), axis image
    colormap gray;
    set(gca, 'CLim', [-1,1]);

    err = err(:);

end

