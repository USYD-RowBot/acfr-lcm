if true
  mu    = TheJournal.Ekf.mu;
  Sigma = TheJournal.Ekf.Sigma;

  eta    = TheJournal.Eif.eta;
  Lambda = TheJournal.Eif.Lambda;
  
  xi_i = [1:12]+10*12;
  xj_i = [1:12]+11*12;
  
else
  eta    = randn(40,1);
  Lambda = sprandcov(40,0.2);

  Sigma = Lambda^-1;
  mu    = Sigma*eta;
  
  N = 3;
  xi_i = [1:N] + 0*N;
  xj_i = [1:N] + 1*N;
  
end

mij_plus = markov_blanket(Lambda,[xi_i,xj_i]);
mij_minus = 1:length(Lambda);
mij_minus([xi_i,xj_i,mij_plus]) = [];


% A: p(xi,xj | mij_plus)
M_plus = randn(length(mij_plus),1);
[mu_A,Sigma_A] = margcondgauss_cov(mu,Sigma,[xi_i,xj_i],mij_minus,mij_plus,M_plus);

[eta_A,Lambda_A] = margcondgauss_info(eta,Lambda,[xi_i,xj_i],mij_minus,mij_plus,M_plus);

