%% Multiple View Geometry 2020, Exercise Sheet 1
% Prof. Dr. Florian Bernard, Lukas Köstler, Mohammed Brahimi

%% Exercise 1

% (b)
I = imread('lena.png');

% (c) 读取图片
[rows, cols, channels] = size(I)
imshow(I), axis image

% (d) 变成灰度图， 求最大值 最小值
I_gray = rgb2gray(I);

min_I = min(I_gray(:))
max_I = max(I_gray(:))

% (e)  a two-dimensional filter filt
filt = fspecial('gaussian')
I_double = im2double(I_gray); % needed for convolution

I_filt = conv2(I_double,filt,'same');

imwrite(I_filt,'lena_gauss.png');

% (f)
subplot(131), imshow(I),      title('Original Lena')
subplot(132), imshow(I_gray), title('Grayscale Lena')
subplot(133), imshow(I_filt), title('Smoothed Lena')

% (g)
% Choose other parameters in the fspecial function

%% Exercise 2

% (a)
A = [2 2 0; 0 8 3]
b = [5; 15]

x = A\b

% (b)
B = A

% (c)
A(1,2) = 4

% (d)
c = 0;
for i=-4:4:4
    c = c + i*A'*b;
end
c

% (e)
A.*B % is an element-wise multiplication, requires matrices of equal size
A'*B % is the matrix multiplication, requires number of columns of first factor = number of rows of second one

%% Exercise 3

approxequal([1,2;3,4], [1,2;3.1,3.5], 0.5)

%% Exercise 4

addprimes(1,10)
addprimes2(1,10)