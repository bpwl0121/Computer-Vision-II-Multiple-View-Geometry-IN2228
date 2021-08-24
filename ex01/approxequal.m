function [out] =approxequal(x,y,ep)
    out=all(abs(x-y))<ep
end