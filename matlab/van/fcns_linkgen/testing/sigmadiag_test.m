function Sigma = sigmadiag_test(Lambda,Xf_ii)

Naug = length(Lambda);
Nf = length(Xf_ii);
Sigma = spalloc(Nf,Nf,6*6*Nf);

kk = 1:6;
for ii=1:Nf
  Xf_i = Xf_ii{ii};
  Icol = spalloc(Naug,6,6);
  Icol(Xf_i(1:6),:) = speye(6);
  Sigma_col = Lambda \ Icol;
  Sigma(kk,kk) = Sigma_col(Xf_i(1:6),:);
  kk = kk+6;
end
