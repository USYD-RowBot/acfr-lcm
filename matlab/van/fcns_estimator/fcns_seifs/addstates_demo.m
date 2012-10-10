clear all;

n = 12; % number of vehicle states
xi = 1:n;

P_plus = randcov(n);
u_plus = randn(n,1);

H_plus = symmetrize(P_plus^-1);
n_plus = H_plus*u_plus;

fu = u_plus(xi);
F  = eye(n);
Q  = randcov(n);

K = 1;
for k=1:10
  disp(k);
  [n_plus,H_plus] = init_dstate_infoform(n_plus,H_plus,xi,u_plus,fu,F,Q);
  [u_plus,P_plus] = init_dstate_covform(u_plus,P_plus,xi,fu,F,Q);

  N = length(u_plus);
  
  % only keep every Kth state
  if false && mod(k,K) ~= 0
    [n_plus,H_plus] = marggauss_info(n_plus,H_plus,N-n+1:N);
    [u_plus,P_plus] = marggauss_cov(u_plus,P_plus,N-n+1:N);
  end
end
