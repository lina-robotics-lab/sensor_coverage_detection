% Matlab uses 1-based indexing, which makes cyclic array indexing by a[mod(i,p)]not
% directly available. But we can hack our way out as follows:
function r=cyclic_mod(n,p)
    r = p-mod(-n,p);
end
% The function behavior is the following: cyclic_mod(1,3)=1,cyclic_mod(2,3)=2, cyclic_mod(3,3)=3,
% cyclic_mod(4,3)=1,cyclic_mod(5,3)=2...
