function [ Id, Dd, Kd ] = downscale( I, D, K, level )
    % downscale intensity image, depth map and camera intrinsics (recursively)

    if(level <= 1)
        % coarsest pyramid level
        Id = I;
        Dd = D;
        Kd = K;
        return;
    end

    % downscale camera intrinsics
    % TODO
    %Kd = ...

    % downscale intensity image
    % TODO
    %Id = ...

    % downscale depth map
    % TODO
    %Dd = ...

    % recursively downscale on next pyramid level
    [Id, Dd, Kd] = downscale( Id, Dd, Kd, level - 1);    
end