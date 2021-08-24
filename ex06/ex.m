image1 = double(imread('batinria0.tif'));
image2 = double(imread('batinria1.tif'));
[x1,y1,x2,y2] = getpoints2()

K1 = [844.310547 0 243.413315; 0 1202.508301 281.529236; 0 0 1];
K2 = [852.721008 0 252.021805; 0 1215.657349 288.587189; 0 0 1];
K1 = inv(K1);
K2 = inv(K2);
nPoints = 12;
% Transform image coordinates with inverse camera matrices:
% can be done without for-loop as well
for i = 1:nPoints
   l = K1 * [x1(i); y1(i); 1];
   r = K2 * [x2(i); y2(i); 1];
   x1(i) = l(1);
   y1(i) = l(2);
   x2(i) = r(1);
   y2(i) = r(2);
end

% Compute constraint matrix chi:
chi = zeros(nPoints,9);
for i = 1:nPoints
    chi(i,:) = kron([x1(i) y1(i) 1],[x2(i) y2(i) 1])';
end
rank_chi = rank(chi)

% Find minimizer for |chi*E|:
[UChi,DChi,VChi] = svd(chi);

% Unstacked ninth column of V:
E =chi*VChi(:,9)

% SVD of E
[U,D,V] = svd(E);
if det(U) < 0 || det(V) < 0
    [U,D,V] = svd(-E);
end

function A = hat(v)
    A = [0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0];
end

function [x1,y1,x2,y2] = getpoints2()

x1 = [
   10.0000
   92.0000
    8.0000
   92.0000
  289.0000
  354.0000
  289.0000
  353.0000
   69.0000
  294.0000
   44.0000
  336.0000
  ];

y1 = [ 
  232.0000
  230.0000
  334.0000
  333.0000
  230.0000
  278.0000
  340.0000
  332.0000
   90.0000
  149.0000
  475.0000
  433.0000
    ];
 
x2 = [
  123.0000
  203.0000
  123.0000
  202.0000
  397.0000
  472.0000
  398.0000
  472.0000
  182.0000
  401.0000
  148.0000
  447.0000
    ];

y2 = [ 
  239.0000
  237.0000
  338.0000
  338.0000
  236.0000
  286.0000
  348.0000
  341.0000
   99.0000
  153.0000
  471.0000
  445.0000
    ];

end

