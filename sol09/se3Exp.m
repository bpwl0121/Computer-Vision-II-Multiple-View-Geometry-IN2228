function [ T ] = se3Exp( twist )
% twist se(3)转SE(3) 
M = [0 -twist(6) twist(5) twist(1);
     twist(6) 0 -twist(4) twist(2);
     -twist(5) twist(4) 0 twist(3);
     0 0 0 0];
T = expm(M);
end

