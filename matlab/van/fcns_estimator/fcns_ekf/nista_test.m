function [ebar,r1,r2] = nista_test(nu,S)

[nz,K] = size(nu);

nis_sum = 0;
for k=1:K
  nis_sum = nis_sum + nu(:,k)'*S(:,:,k)^-1*nu(:,k);
end
ebar = nis_sum/K;

% two-sided chi-squared with K*nx DOF acceptance bounds
r1 = chi2inv(0.0275,K*nz)/K;
r2 = chi2inv(0.9750,K*nz)/K;
