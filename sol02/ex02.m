% solve for the best fitting linear function of several variables 
% through a set of points using the Matlab function svd
  

%% create the data

% generate data set consisting of m samples of each of 4 variables
% d1，d2，d3，d4都是三行一列的向量
m = 3;
d1 = rand(m,1);
d2 = rand(m,1);
d3 = rand(m,1);
d4 = 4*d1 - 3*d2 + 2*d3 - 1;

% introduce small errors into the data
% (the 'rand' function produces numbers between 0 and 1)
eps = 1.e-4;
d1 = d1 .* (1 + eps * rand(m,1));
d2 = d2 .* (1 + eps * rand(m,1));
d3 = d3 .* (1 + eps * rand(m,1));
d4 = d4 .* (1 + eps * rand(m,1));

%% find the coefficients x solving the system: x1*d1 + x2*d2 + x3*d3 + x4*d4 = 1;
%  D是m×n，伪逆D^+是n×m,
% define A and compute the svd
D = [d1 d2 d3 d4];
[U, S, V] = svd(D);

% construct b and S^+，
% S_plus就是构造一个n×m矩阵，然后对角线是原来非0特征值的倒数
% 先转置，再把非0元素变成倒数就行，因为只有对角线才有非0元素
S_plus = S';
S_plus(S_plus ~= 0) = S_plus(S_plus ~= 0) .^ -1;
b = ones(m,1);

% solve the least squares problem using the pseudo inverse D_plus = V * S_plus * U'
D_plus = V * S_plus * U';
D_plus_matlab = pinv(D);
disp(abs(D_plus - D_plus_matlab)) % allow to compare our pseudo inverse and the one using matlab pinv function
%这里求解的x其实是欠定方程的解，matlab得到的解是norm最小的欠定方程的解！
x = D_plus * b;
disp(x)


%% When m = 3, display the graph of the norm of x_lambda and the error according to lambda to check if the pseudo inverse solution has the minimal norm among all solutions.
if (m==3)
    
    v = null(D); % get a vector of the kernel of D
    norm(v)
    % lambda 是从-100到100的数组
    lambda = -100:1:100;
    % nb_values是lambda的长度
    nb_values = length(lambda);
    values_norm = zeros(nb_values,1);
    values_error = zeros(nb_values,1);
    
    for i = 1:nb_values
        % S，x已经是解，v是kernel的向量，x_lambda是二者的和，还是解
        x_lambda = x + lambda(i)*v;
        values_norm(i) = norm(x_lambda);
        values_error(i) = norm(D*x_lambda - b)^2;
    end
    
    %Display the graph, we notice that all x_lambda have the same error, and for lambda = 0, we have the smallest
    %norm possible, which was expected.
    
    % 图像里有两条线，一条蓝色，是lambda和x的norm的关系，随着lambda绝对值的变换，x的norm在变化
    %还有一条红色，是lambda和 values_error的关系，几乎是0，说明，lambda不影响解，因为v是kernel
    % 根据图像，当lambda是0的时候，x + lambda(i)*v;有最小范数，也就是说，x本身是norm最小的欠定方程的解
    figure(1)
    plot(lambda,values_norm,'b-',lambda,values_error,'r-');
    
end

%% Notes:

% In Matlab you would usually solve a linear system with the built-in solver:
% x = D\b;
