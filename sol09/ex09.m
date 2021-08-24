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
useGN = false; % Gauss-Newton
useLM = true; % Levenberg Marquardt
useGD = false; % Gradient descent

% load dataset
if (dataset == 1)
    % camera intrinsics
    K = [517.3 0 318.6;	0 516.5 255.3; 0 0 1];
    % first pair of input frames
    c1 = double(imreadbw('rgb/1305031102.275326.png'));
    c2 = double(imreadbw('rgb/1305031102.175304.png'));
%     c2 = double(imreadbw('rgb/1305031102.175304_broken.png'));
    d1 = double(imread('depth/1305031102.262886.png'))/5000;
    d2 = double(imread('depth/1305031102.160407.png'))/5000;
else
    % camera intrinsics
    K = [ 535.4  0 320.1;	0 539.2 247.6; 0 0 1];
    % second pair of input frames
%     c1 = double(imreadbw('rgb/1341847980.722988.png'));
    c1 = double(imreadbw('rgb/1341847980.722988_broken.png'));
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
    lambda = 0.1;
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
        % Huber的权重
        huber = ones(size(residual));
        if (useHuber)
            % compute Huber Weights
            huberDelta = 4/255;
            huber(abs(residual) > huberDelta) = huberDelta ./ abs(residual(abs(residual) > huberDelta));

            % plot Huber Weights
            subplot(1,2,2);
            imagesc(reshape(huber, size(I)))
            axis equal
        end
        % compute weighted residuals by applying Huber weights
        residualHuber = huber .* residual;

        if(useGN)
            % do Gauss-Newton step
            % repmat(huber,1,6)，huber原来是n×1，现在是n×6，复制成6列，
            % (repmat(huber,1,6) .* Jac) 是element wise的乘法，都是n×6
            % Jac' * (repmat(huber,1,6) .* Jac) 是6×n n×6，变成6×6
            upd = - (Jac' * (repmat(huber,1,6) .* Jac))^-1 * Jac' * residualHuber;
        end

        if (useGD)
            % do gradient descent
            upd = - Jac' * residualHuber;
            upd = 0.001 * upd / norm(upd)   % choose step size such that the step is always 0.001 long.
        end

        if (useLM)
            % do LM
            H = (Jac' * (repmat(huber,1,6) .* Jac));
            % diag(H)只能取对角线的元素，
            %diag(diag(H))把取好的对角线元素再构成对角矩阵，有lambda的参数
            %有lambda的参数，后面会不断优化
            upd = - (H + lambda * diag(diag(H)))^-1 * Jac' * residualHuber;
        end


        % multiply increment from left onto the current estimate
        lastXi = xi;
        % exp(ϕ∧1)exp(ϕ∧2)=exp((ϕ1+ϕ2)∧) 不成立！！！！！！
        % Baker-Campbell-Hausdorff公式
        xi = se3Log(se3Exp(upd) * se3Exp(xi));
        xi'

        % get mean and display
        % 这个err是上一步，还没更新xi的err
        err = mean(residualHuber .* residual)
        %calcErr(c1,d1,c2,xi,K);

        if (useLM)
            %err - errLast
            if (err >= errLast)
                % error increased -> increase lambda
                lambda = lambda * 5
                xi = lastXi;

                if(lambda > 5)
                    % early break
                    break;
                end
            else
                % error decreased -> decrease lambda
                lambda = lambda / 1.5
            end
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
