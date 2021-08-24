function [ Jac, residual ] = deriveAnalytic( IRef, DRef, I, xi, K )
    % calculate analytic derivative

    % get shorthands (R, t)
    T = se3Exp(xi);
    R = T(1:3, 1:3);
    t = T(1:3,4);

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
            % TODO warp points into target frame
        end
    end


    % ========= calculate actual derivative. ===============
    % 1.: calculate image derivatives, and interpolate at warped positions.
    % TODO image gradient in x and y direction using central differences
    %dxI = ...
    %dyI = ...
    % interpolate at warped positions
    Ixfx = K(1,1) * reshape(interp2(dxI, nImg+1, mImg+1),size(I,1) * size(I,2),1);
    Iyfy = K(2,2) * reshape(interp2(dyI, nImg+1, mImg+1),size(I,1) * size(I,2),1);

    % 2.: get warped 3d points (x', y', z').
    xp = xp(:);
    yp = yp(:);
    zp = zp(:);

    % 3. implement gradient computed in Theory Ex. 1 (b)
    Jac = zeros(size(I,1) * size(I,2),6);
    % TODO implement analytic partial derivatives
    %Jac(:,1) = ...
    %Jac(:,2) = ...
    %Jac(:,3) = ...
    %Jac(:,4) = ...
    %Jac(:,5) = ...
    %Jac(:,6) = ...

    % ========= plot residual image =========
    residual = interp2(I, nImg+1, mImg+1) - IRef;
    imagesc(residual), axis image
    colormap gray
    set(gca, 'CLim', [-1,1])
    residual = residual(:);
end

