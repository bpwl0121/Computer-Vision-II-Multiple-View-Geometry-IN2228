function [ result ] = addprimes( s, e )
    z = s:e;
    % isprime得到全部prime数的下标，
    % z(isprime(z)) 取得数字，最后sum
    result = sum(z(isprime(z)));
end

