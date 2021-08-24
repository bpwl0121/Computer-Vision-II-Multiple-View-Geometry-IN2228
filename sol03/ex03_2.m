%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTES:
% - The implementations follow the notation of Ethan Eade's tech report,
%   which is a concise reference for various relevant Lie groups. Simple
%   arithmetic manipulation (using the properties of w_hat discussed in the
%   tutorial) shows the equivalence to the formulation on the lecture
%   slides. Of course you can also implement those variants.
% - Taylor expansions are used for small theta to avoid numerical issues of
%   dividing by something close to 0. This is not strictly needed, but at
%   least a check for theta != 0 should be added.
%
% Additional ressources on common Lie Groups used in 3D computer vision:
%  - Lie Groups for 2D and 3D Transformations (Ethan Eade)
%  - A tutorial on SE(3) transformation parameterizations and on-manifold optimization (Jose-Luis Blanco)

close all;

%% setup

% try large and small vector (to test Taylor expansion for small angles)

w1 = rand(3,1);
v1 = rand(3,1);
w2 = w1/norm(w1) * 1e-6;
v2 = v1/norm(v1) * 1e-6;
xi1 = [v1;w1];
xi2 = [v2;w2];

%% SO(3)

% compute exponential
R1 = Exp_SO3(w1);
R2 = Exp_SO3(w2);

% compute logarithm to get back to the original value
w1_b = Log_SO3(R1);
w2_b = Log_SO3(R2,0);

% compare original w with log(exp(w)) --> compute relative difference
% we expect small values (1e-10 or smaller)
test_exp_log_so3_1 = norm(w1 - w1_b) / norm(w1)
test_exp_log_so3_2 = norm(w2 - w2_b) / norm(w2)

% Compare to Matlab's generic expm function --> compute relative difference
% we expect small values (1e-10 or smaller)
test_exp_so3_1 = norm(R1 - expm(hat_SO3(w1))) / norm(w1)
test_exp_so3_2 = norm(R2 - expm(hat_SO3(w2))) / norm(w2)
test_log_so3_1 = norm(w1_b - vee_SO3(logm(R1))) / norm(w1)
test_log_so3_2 = norm(w2_b - vee_SO3(logm(R2))) / norm(w2)


%% SE(3)

% compute exponential
T1 = Exp_SE3(xi1);
T2 = Exp_SE3(xi2);

% compute logarithm to get back to the original value
xi1_b = Log_SE3(T1);
xi2_b = Log_SE3(T2);

% compare original w with log(exp(w)) --> compute relative difference
% we expect small values (1e-10 or smaller)
test_exp_log_se3_1 = norm(xi1 - xi1_b) / norm(xi1)
test_exp_log_se3_2 = norm(xi2 - xi2_b) / norm(xi2)

% Compare to Matlab's generic expm function --> compute relative difference
% we expect small values (1e-10 or smaller)
test_exp_se3_1 = norm(T1 - expm(hat_SE3(xi1))) / norm(xi1)
test_exp_se3_2 = norm(T2 - expm(hat_SE3(xi2))) / norm(xi2)
test_log_se3_1 = norm(xi1_b - vee_SE3(logm(T1))) / norm(xi1)
test_log_se3_2 = norm(xi2_b - vee_SE3(logm(T2))) / norm(xi2)





%% Bonus: Section about epsilon and Taylor expansion
% This explores the effect of using the Taylor expansion for small inputs
% by comparing to expm / logm. This is not a perfect test, since we are not
% certain about the accuracy of expm / logm. It can be seen that the Taylor
% expansion is only valid for small angles (as expected). For exp we can 
% also clearly see that for input norms around 1e-4 or smaller the Taylor 
% expansion seems to be more consistent with expm. For log, maybe the 
% Taylor expansion is less relevant, as both seem to be consistent with 
% logm for small values.

figure(1)
test_taylor_exp_so3();
figure(2)
test_taylor_exp_se3();
figure(3)
test_taylor_log_so3();
figure(4)
test_taylor_log_se3();



%% implementation:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hat and vee (inverse of hat) operators for SO(3) and SE(3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% skew symmetric matrix, so(3) 到skew symmetric matrix。 S是w_hat
function S = hat_SO3(w)
    S = [  0   -w(3)  w(2)
          w(3)   0   -w(1)
         -w(2)  w(1)   0  ];
end

% return the special vector from SO(3), to build the rotation vector???
% so(3)到rotation vector, w是转轴向量
function w = vee_SO3(S)
    % check S is skew symmetric
    assert(norm(S+S') < 1e-10)
    
    w = [S(3,2); S(1,3); S(2,1)];
end

%build twist se(3) from twist coordinate ，Xi是se（3），还要过exp才是R，T
function Xi = hat_SE3(xi)
    v = xi(1:3);
    w = xi(4:6);
    Xi = zeros(4);
    Xi(1:3,1:3) = hat_SO3(w);
    Xi(1:3,4) = v;
end

%从Xi到xi，hat_SE3的反操作，把se（3）变成twist
function xi = vee_SE3(Xi)
    S = Xi(1:3,1:3);
    v = Xi(1:3,4);
    
    % check last row is 0
    assert(norm(Xi(4,:)) < 1e-10);
    
    % this checks if S is skew symmetric
    w = vee_SO3(S);
    
    xi = [v; w];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exponential for so(3) --> SO(3)
% exp[w_hat] = I + sin(|w|)/|w| * w_hat + (1-cos(|w|))/|w| * w_hat^2
% Compare:
%  - Lie Groups for 2D and 3D Transformations (Ethan Eade) p4
%  - Ex. 3 Part I 3.
%  - Lecture Ch. 2, p15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exponential map from so(3) to SO(3)
% 从w转R旋转矩阵
function R = Exp_SO3(w, epsilon)
    if ~exist('epsilon','var')
        % set default value for epsilon
        epsilon = 1e-5;
    end
    % w到w hat
    S = hat_SO3(w);
    theta_2 = w'*w;
    theta = sqrt(theta_2);
    % Use Taylor expansion to avoid numerical instabilities for small theta
    % At least a check for theta != 0 would be needed.
    if theta <= epsilon
        % http://www.wolframalpha.com/input/?i=sin(theta)%2Ftheta
        A = 1 - theta_2/6;
        % https://www.wolframalpha.com/input/?i=(1+-+cos(theta))%2Ftheta%5E2
        B = 0.5 - theta_2/24;
    else
        %theta=|w|
        A = sin(theta)/theta;
        B = (1 - cos(theta))/theta_2;
    end
    % Rodrigues' formula
    R = eye(3) + A*S + B*S*S;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Logarithm for SO(3) --> so(3)
% |w| = acos((trace(R)-1)/2)
% w = |w| / (2*sin(|w|)) * vee(R-R')
% Compare:
%  - Lie Groups for 2D and 3D Transformations (Ethan Eade) p4
%  - Lecture Ch. 2, p13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% from SO(3) to so(3), the vector 
%从R旋转矩阵到so3
function w = Log_SO3(R, epsilon)
    if ~exist('epsilon','var')
        % set default value for epsilon
        epsilon = 1e-5;
    end
     %theta=|w|
    theta = acos((trace(R)-1)/2);
    % Use Taylor expansion to avoid numerical instabilities for small theta
    % At least a check for theta != 0 would be needed.
    if theta <= epsilon
        % http://www.wolframalpha.com/input/?i=theta+%2F+sin(theta)
        A = 1 + (theta^2)/6;
    else
        A = theta / sin(theta);
    end
    %vee_SO3是 第二章13页
    w = 0.5 * A * vee_SO3(R-R');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exponential for se(3) --> SE(3)
%               [ exp[w_hat]   V*v ]
% exp[xi_hat] = [                  ]
%               [      0        1  ]
% V = ((I - exp[w_hat])*w_hat + w*w')/|w|^2
% Equivalent formulation:
% V = I + B*w_hat + C*w_hat^2
% B = (1-cos(|w|)) / |w|^2
% C = (1 - sin(|w|)/|w|) / |w|^2
% Compare:
%  - Lie Groups for 2D and 3D Transformations (Ethan Eade) p9
%  - Lecture Ch. 2, p19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% xi twist coordinate 
% from twist coordinate to SE(3)
function T = Exp_SE3(xi, epsilon)
    if ~exist('epsilon','var')
        % set default value for epsilon
        epsilon = 1e-5;
    end
    v = xi(1:3);
    w = xi(4:6);
    S = hat_SO3(w);
    S_2 = S*S;
    theta_2 = w'*w;
    theta = sqrt(theta_2);
    % Use Taylor expansion to avoid numerical instabilities for small theta
    % At least a check for theta != 0 would be needed.
    if theta <= epsilon
        % http://www.wolframalpha.com/input/?i=sin(theta)%2Ftheta
        A = 1 - theta_2/6;
        % https://www.wolframalpha.com/input/?i=(1+-+cos(theta))%2Ftheta%5E2
        B = 0.5 - theta_2/24;
        % http://www.wolframalpha.com/input/?i=(1+-+sin(theta)%2Ftheta)+%2F+theta%5E2
        C = 1/6 - theta_2/120;
    else
        A = sin(theta)/theta;
        B = (1 - cos(theta))/theta_2;
        C = (1 - A) / theta_2;
    end
    %S = hat_SO3(w);
    R = eye(3) + A*S + B*S_2;  % Rodrigues' formula
    % 这里用了不一样的求T的方法，书里的方法是V2
    V1 = eye(3) + B*S + C*S_2;
    V2 = ((eye(3) -R)*S+w*w')/(norm(w)*norm(w));
    if V1==V2
        fprintf('job done \n')
    end
    V=V1
    T = eye(4);
    T(1:3,1:3) = R;
    % 最后这里×v
    T(1:3,4) = V*v;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Logarithm for SE(3) --> se(3)
%      [ v = inv(V)*t    ]
% xi = [                 ]
%      [ w = vee(log(R)) ]
% V = ((I - exp[w_hat])*w_hat + w*w')/|w|^2
% Note: "inv" is the matrix inverse
% Equivalent closed-form solution:
% inv(V) = eye(3) - 0.5*S + D*S^2
% D = (1 - (sin(|w|)*|w|)/(2*(1 - cos(|w|)))) / |w|^2
% Compare:
%  - Lecture Ch. 2, p19 (doesn't give closed-form inverse of V)
%  - Lie Groups for 2D and 3D Transformations (Ethan Eade) p9/10 
%    (includes closed-form inverse of V)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 从SE3到twist
function xi = Log_SE3(T, epsilon)
    if ~exist('epsilon','var')
        % set default value for epsilon
        epsilon = 1e-5;
    end
    R = T(1:3,1:3);
    t = T(1:3, 4);
    assert(norm(T(4,:) - [0 0 0 1]) < 1e-10);
    theta = acos((trace(R)-1)/2);
    theta_2 = theta^2;
    % Use Taylor expansion to avoid numerical instabilities for small theta
    % At least a check for theta != 0 would be needed.
    % 这里是否用泰勒级数，theta小于epsilon就用
    if theta <= epsilon
        % http://www.wolframalpha.com/input/?i=theta+%2F+sin(theta)
        A = 1 + (theta_2)/6;
        % http://www.wolframalpha.com/input/?i=(1+-+(sin(theta)%2Ftheta)%2F(2*(1+-+cos(theta))%2Ftheta%5E2))%2Ftheta%5E2
        D = 1/12 - theta_2/720;
    else
        A = theta / sin(theta);
        A_temp = sin(theta)/theta;
        B_temp = (1 - cos(theta))/theta_2;
        D = (1 - A_temp/(2*B_temp))/theta_2;
    end

    % w = 0.5 * A * vee_SO3(R-R');
    % 之前是先vee_SO3(R-R')，一样
    S = 0.5 * A * (R-R');
    %求得w
    w = vee_SO3(S);
    S_2 = S*S;
    w_hat = hat_SO3(w);
    % 这个V2是V2*V＝t
    V2 = ((eye(3) -R)*w_hat+w*w')/(norm(w)*norm(w));
    v2= inv(V2)*t
    % Note: It is not obvious to derive the closed-form solution of V, and
    % one possibility is to use Matlab's general matrix inverse inv(V)
    % to compute it (but the closed-form solution is probably faster).
    V_inv = eye(3) - 0.5*S + D*S_2;
    v = V_inv * t;
    if v2==v
         fprintf('jobdssssssssssssssssssss \n')
    end
    xi = [v; w];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Taylor expansion tests:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test_taylor_exp_so3()

    n = 20;

    x = zeros(n,1);
    diff_taylor = zeros(n,1);
    diff_no_taylor = zeros(n,1);
    
    w0 = rand(3,1);
    w0 = 100 * w0 / norm(w0);

    for i = 1:n
        w = w0 / 10^i;
        x(i) = norm(w);
        % epsilon 是inf，必用泰勒级数，
        diff_taylor(i) = norm(Exp_SO3(w, inf) - expm(hat_SO3(w)),'fro');
        % epsilon是0，不用泰勒级数
        diff_no_taylor(i) = norm(Exp_SO3(w, 0) - expm(hat_SO3(w)),'fro');
    end

    loglog(x, diff_taylor./x, 'g')
    hold on
    loglog(x, diff_no_taylor./x, 'r')
    legend('Taylor', 'no Taylor')
    title('Exp_{SO3}(w) vs expm(hat_{SO3}(w))', 'Interpreter', 'none')
    
end

function test_taylor_exp_se3()

    n = 20;

    x = zeros(n,1);
    diff_taylor = zeros(n,1);
    diff_no_taylor = zeros(n,1);
    
    xi0 = rand(6,1);
    xi0 = 100 * xi0 / norm(xi0);

    for i = 1:n
        xi = xi0 / 10^i;
        x(i) = norm(xi);
        diff_taylor(i) = norm(Exp_SE3(xi, inf) - expm(hat_SE3(xi)),'fro');
        diff_no_taylor(i) = norm(Exp_SE3(xi, 0) - expm(hat_SE3(xi)),'fro');
    end

    loglog(x, diff_taylor./x, 'g')
    hold on
    loglog(x, diff_no_taylor./x, 'r')
    legend('Taylor', 'no Taylor')
    title('Exp_{SE3}(xi) vs expm(hat_{SE3}(xi))', 'Interpreter', 'none')
    
end

function test_taylor_log_so3()

    n = 20;

    x = zeros(n,1);
    diff_taylor = zeros(n,1);
    diff_no_taylor = zeros(n,1);
    
    w0 = rand(3,1);
    w0 = 10 * w0 / norm(w0);

    for i = 1:n
        w = w0 / 10^i;
        x(i) = norm(w);
        R = expm(hat_SO3(w));
        diff_taylor(i) = norm(expm(hat_SO3(Log_SO3(R, inf))) - R,'fro');
        diff_no_taylor(i) = norm(expm(hat_SO3(Log_SO3(R, 0))) - R,'fro');
    end

    loglog(x, diff_taylor./x, 'g')
    hold on
    loglog(x, diff_no_taylor./x, 'r')
    legend('Taylor', 'no Taylor')
    title('Log_{SO3}(R) vs vee_{SO3}(logm(R))', 'Interpreter', 'none')
    
end

function test_taylor_log_se3()

    n = 20;

    x = zeros(n,1);
    diff_taylor = zeros(n,1);
    diff_no_taylor = zeros(n,1);
    
    xi0 = rand(6,1);
    xi0 = 10 * xi0 / norm(xi0);

    for i = 1:n
        xi = xi0 / 10^i;
        x(i) = norm(xi);
        T = expm(hat_SE3(xi));
        diff_taylor(i) = norm(expm(hat_SE3(Log_SE3(T, inf))) - T,'fro');
        diff_no_taylor(i) = norm(expm(hat_SE3(Log_SE3(T, 0))) - T,'fro');
    end

    loglog(x, diff_taylor./x, 'g')
    hold on
    loglog(x, diff_no_taylor./x, 'r')
    legend('Taylor', 'no Taylor')
    title('Log_{SE3}(T) vs vee\_{SE3}(logm(T))', 'Interpreter', 'none')
    
end

