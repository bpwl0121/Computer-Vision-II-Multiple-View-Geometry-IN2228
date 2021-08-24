function V = transform(V, angles, t)
    % rotate the vertices V around their center with given angles around 
    % axes x, y and z, then translate by t
    
    centroid = mean(V)';
    t_zero = zeros(3, 1);
    % 顺序是从右到左
    % right-to-left: shift to center, rotate by x, y, z, shift back,
    % translate by translation
    % angles(1)是x的角度，angles(2)是y的角度，angles(3)是z的角度
    T = SE3(eye(3), t) * ...
        SE3(eye(3), centroid) * ... 
        SE3(rotmatz(angles(3)), t_zero) * ...
        SE3(rotmaty(angles(2)), t_zero) * ...
        SE3(rotmatx(angles(1)), t_zero) * ...
        SE3(eye(3), -centroid);
    % Vhom 是V的homogeneous coordinate，相当于加了一个维度，值是1
    Vhom = [V ones(size(V, 1), 1)]';
    Vhom = T * Vhom;
    
    % We can simply drop the last element since it will still be 1 in this
    % case. In general to convert to euclidian coordinates you need to
    % divide by the last element.
    V = Vhom(1:3,:)';
end

function T = SE3(R, t)
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = t;
end

function R = rotmatx(a)
    R = [1      0       0;
         0 cos(a) -sin(a);
         0 sin(a)  cos(a)];
end

function R = rotmaty(a)
    R = [ cos(a) 0 sin(a);
               0 1      0;
         -sin(a) 0 cos(a)];
end

function R = rotmatz(a)
    R = [cos(a) -sin(a) 0;
         sin(a)  cos(a) 0;
              0       0 1];
end