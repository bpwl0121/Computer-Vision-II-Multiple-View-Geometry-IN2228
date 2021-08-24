A = [2 2 0; 0 8 3]
B=A
b = [5; 15]
x=A/B
c=0;
for i=-4:4:4
    c=c+i*A'*b;
end
c