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
    xp = NaN(size(IRef));
    yp = NaN(size(IRef));
    zp = NaN(size(IRef));
    for n=1:size(IRef,2)
        for m=1:size(IRef,1)

            % point in reference image. note that the pixel-coordinates of the
            % point (1,1) are actually (0,0).
            p = [n-1;m-1;1] * DRef(m,n);

            % transform to image (unproject, rotate & translate)
            pTrans = RKInv * p + t;

            % if point is valid (depth > 0), project and save result.
            if(pTrans(3) > 0 && DRef(m,n) > 0)
                % projected point (for interpolation of intensity and gradients)
                pTransProj = K * pTrans;
                nImg(m,n) = pTransProj(1) / pTransProj(3);
                mImg(m,n) = pTransProj(2) / pTransProj(3);

                % warped 3d point, for calculation of Jacobian.
                xp(m,n) = pTrans(1);
                yp(m,n) = pTrans(2);
                zp(m,n) = pTrans(3);
            end
        end
    end


    % ========= calculate actual derivative. ===============
    % 1.: calculate image derivatives, and interpolate at warped positions.
    dxI = NaN(size(I));
    dyI = NaN(size(I));
    dyI(2:(end-1),:) = 0.5*(I(3:(end),:) - I(1:(end-2),:));
    dxI(:,2:(end-1)) = 0.5*(I(:,3:(end)) - I(:,1:(end-2)));
    Ixfx = K(1,1) * reshape(interp2(dxI, nImg+1, mImg+1),size(I,1) * size(I,2),1);
    Iyfy = K(2,2) * reshape(interp2(dyI, nImg+1, mImg+1),size(I,1) * size(I,2),1);

    % 2.: get warped 3d points (x', y', z').
    xp = xp(:);
    yp = yp(:);
    zp = zp(:);

    % 3. implement gradient computed in Theory Ex. 1 (b)
    Jac = zeros(size(I,1) * size(I,2),6);
    Jac(:,1) = Ixfx ./ zp;
    Jac(:,2) = Iyfy ./ zp;
    Jac(:,3) = - (Ixfx .* xp + Iyfy .* yp) ./ (zp .* zp);
    Jac(:,4) = - (Ixfx .* xp .* yp) ./ (zp .* zp) - Iyfy .* (1 + (yp ./ zp).^2);
    Jac(:,5) = + Ixfx .* (1 + (xp ./ zp).^2) + (Iyfy .* xp .* yp) ./ (zp .* zp);
    Jac(:,6) = (- Ixfx .* yp + Iyfy .* xp) ./ zp;

    % ========= plot residual image =========
    residual = interp2(I, nImg+1, mImg+1) - IRef;
    imagesc(residual), axis image
    colormap gray
    set(gca, 'CLim', [-1,1])
    residual = residual(:);
end

