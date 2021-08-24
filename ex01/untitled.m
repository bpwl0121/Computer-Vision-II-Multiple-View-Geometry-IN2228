f=imread("lena.png")
f_gary=rgb2gray(f)

x=size(f)
f_gary_max=max(f_gary(:))
f_gary_min=min(f_gary(:))
h = fspecial('gaussian')
Iblur1 = imgaussfilt(f_gary,2);
imshow(Iblur1)