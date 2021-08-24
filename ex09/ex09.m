%% Multiple View Geometry 2020, Exercise Sheet 9
% Prof. Dr. Florian Bernard, Lukas Koestler, Mohammed Brahimi

% Direct Image Alignment
% Code by R. Maier, J. Engel

% select dataset
dataset = 1;
% 1 = fr1/xyz:
% expected result approximately  -0.0018    0.0065    0.0369   -0.0287   -0.0184   -0.0004
% 2 = fr3/long_office_household:
% expected result approximately  0.2979   -0.0106    0.0452   -0.0041   -0.0993   -0.0421

% use numeric/analytic derivatives (true=numeric, false=analytic)
useNumeric = false;

% use huber weights
useHuber = true;

% exactly one of those should be true.
useGN = true; % Gauss-Newton
useLM = false; % Levenberg Marquardt
useGD = false; % Gradient descent

% load dataset
if (dataset == 1)
    % camera intrinsics
    K = [517.3 0 318.6;	0 516.5 255.3; 0 0 1];
    % first pair of input frames
    c1 = double(imreadbw('rgb/1305031102.275326.png'));
    c2 = double(imreadbw('rgb/1305031102.175304.png'));
    d1 = double(imread('depth/1305031102.262886.png'))/5000;
    d2 = double(imread('depth/1305031102.160407.png'))/5000;
else
    % camera intrinsics
    K = [ 535.4  0 320.1;	0 539.2 247.6; 0 0 1];
    % second pair of input frames
    c1 = double(imreadbw('rgb/1341847980.722988.png'));
    %c1 = double(imreadbw('rgb/1341847980.722988_broken.png'));
    c2 = double(imreadbw('rgb/1341847982.998783.png'));
    d1 = double(imread('depth/1341847980.723020.png'))/5000;
    d2 = double(imread('depth/1341847982.998830.png'))/5000;
end

% initialization
xi = [0 0 0 0 0 0]';

% pyramid levels
for lvl = 5:-1:1
    lvl

    % downscale reference frame (intensity image, depth map and K-matrix)
    [IRef, DRef, Klvl] = downscale(c1,d1,K,lvl);
    % downscale target frame (intensity image and depth map)
    [I, D] = downscale(c2,d2,K,lvl);

    % just do at most 20 steps.
    errLast = 1e10;

    % init lambda for LM
    % lambda = ...
    for i=1:20
        subplot(1,2,1);

        % calculate Jacobian of residual function (Matrix of dim (width*height) x 6)
        if (useNumeric)
            % use numeric derivatives
            [Jac, residual] = deriveNumeric(IRef,DRef,I,xi,Klvl);
        else
            % use analytic derivatives
            [Jac, residual] = deriveAnalytic(IRef,DRef,I,xi,Klvl);
        end
        axis equal

        % set rows with NaN to 0 (e.g. because out-of-bounds or invalid depth).
        notValid = isnan(sum(Jac,2));
        residual(notValid,:) = 0;
        Jac(notValid,:) = 0;

        % Huber weights
        huber = ones(size(residual));
        if (useHuber)
            % compute Huber Weights
            % huber = ...

            % plot Huber Weights
            subplot(1,2,2);
            imagesc(reshape(huber, size(I)))
            axis equal
        end
        % compute weighted residuals by applying Huber weights
        % residualHuber = ...

        if(useGN)
            % do Gauss-Newton step with Huber weights
            %upd = ...
        end

        if (useGD)
            % do gradient descent with Huber weights
            % TODO
            %upd = ...
        end

        if (useLM)
            % do LM with Huber weights
            % TODO
            %upd = ...
        end

        % multiply increment from left onto the current estimate
        lastXi = xi;
        xi = se3Log(se3Exp(upd) * se3Exp(xi));
        xi'

        % get mean and display
        err = mean(residualHuber .* residual)
        %calcErr(c1,d1,c2,xi,K);

        if (useLM)
            %err - errLast
            % adjust lambda accordingly depending on error decrease/increase
            % TODO
        end

        if (useGN || useGD)
            % early break
            if(err / errLast > 0.995)
                break;
            end
        end

        errLast = err;
    end
end

subplot(122)
imshowpair(IRef,IRef + reshape(residual,size(IRef)))
