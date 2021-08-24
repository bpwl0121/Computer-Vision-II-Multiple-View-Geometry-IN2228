function [ err ] = calcErr( IRef, DRef, I, xi, K )
    % calculate residuals

    % get shorthands (R, t)
    T = se3Exp(xi);
    R = T(1:3, 1:3);
    t = T(1:3,4);

    % these contain the x,y image coordinates of the respective
    % reference-pixel, transformed & projected into the new image.
    % set to -10 initially, as this will give NaN on interpolation later.
    nImg = zeros(size(IRef))-10;
    mImg = zeros(size(IRef))-10;

    % for all pixels
    for n=1:size(IRef,2)
        for m=1:size(IRef,1)
            % TODO warp reference points to target image
            % TODO project warped points onto image
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

