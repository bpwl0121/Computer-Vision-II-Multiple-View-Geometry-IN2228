function [ Jac, residual ] = deriveNumeric( IRef, DRef, I, xi, K )
    % calculate numeric derivative (slow)
    
    % compute residuals for xi
    residual = calcErr(IRef,DRef,I,xi,K);

    % initialize Jacobian
    Jac = zeros(size(I,1) * size(I,2),6);
    
    % compute Jacobian numerically
    eps = 1e-6;
    for j=1:6
        epsVec = zeros(6,1);
        epsVec(j) = eps;
        
        % multiply epsilon from left onto the current estimate.
        xiPerm =  se3Log(se3Exp(epsVec) * se3Exp(xi));
        Jac(:,j) = (calcErr(IRef,DRef,I,xiPerm,K) - residual) / eps;
    end
end

