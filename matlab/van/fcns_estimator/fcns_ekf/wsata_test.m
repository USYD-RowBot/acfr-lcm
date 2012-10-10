function [rho,r1,r2] = wsata_test(nu,maxlag)

[nz,K] = size(nu);

if ~exist('maxlag','var') || isempty(maxlag), maxlag = K-1; end

vv = nu'*nu;


% two-sided chi-squared with K*nx DOF acceptance bounds
r1 = norminv(0.0275,0,1/K);
r2 = norminv(0.9750,0,1/K);

