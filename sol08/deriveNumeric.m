function [ Jac, residual ] = deriveNumeric( IRef, DRef, I, xi, K )
    % calculate numeric derivative (slow)
    
    % compute residuals for xi
    residual = calcErr(IRef,DRef,I,xi,K);

    % initialize Jacobian
    Jac = zeros(size(I,1) * size(I,2),6);
    
    % compute Jacobian numerically
    eps = 1e-6;
    for j=1:6
        % 看题目，e是twist的unite vector，对应e1到e6
        epsVec = zeros(6,1);
        epsVec(j) = eps;
        % e1到e6和xi计算变换后残差，再减去原本残差
        % e1到e6和xi计算变换相当于在xi的基础上再进行变换，在6个维度上
        % 得到每个点在6个维度上的梯度，相当于用了做差法求梯度
        % multiply epsilon from left onto the current estimate.
        xiPerm =  se3Log(se3Exp(epsVec) * se3Exp(xi));
        Jac(:,j) = (calcErr(IRef,DRef,I,xiPerm,K) - residual) / eps;
    end
end

