clear all;

n = 12; % number of vehicle states
xt_i = 1:n;

P_plus = randcov(4*n);
u_plus = randn(4*n,1);

H_plus = symmetrize(P_plus^-1);
n_plus = H_plus*u_plus;

fu = u_plus(xt_i);
F  = eye(n);
Q  = randcov(n);

% augment to create the current robot pose
[n_plus,H_plus] = init_dstate_infoform(n_plus,H_plus,xt_i,u_plus,fu,F,Q);
[u_plus,P_plus] = init_dstate_covform(u_plus,P_plus,xt_i,fu,F,Q);

nn_plus = n_plus;
HH_plus = H_plus;
uu_plus = u_plus;
PP_plus = P_plus;
uuu_plus = uu_plus;

K = 1;
for k=1:100
  disp(k);
  H_old = H_plus;
  % motion predict via augment & marginalize
  [n_plus,H_plus] = init_dstate_infoform(n_plus,H_plus,xt_i,u_plus,fu,F,Q);
  [u_plus,P_plus] = init_dstate_covform(u_plus,P_plus,xt_i,fu,F,Q);

  N = length(u_plus);
  
  [n_plus,H_plus] = marggauss_info(n_plus,H_plus,xt_i+n);
  [u_plus,P_plus] = marggauss_cov(u_plus,P_plus,xt_i+n);

  % direct motion predict
  [nn_plus,HH_plus] = motionpredict_infoform(nn_plus,HH_plus,xt_i,uuu_plus,fu,F,Q);
  uuu_plus = HH_plus^-1*nn_plus;
  [uu_plus,PP_plus] = motionpredict_covform(uu_plus,PP_plus,xt_i,fu,F,Q);
  
  %emeas = log10(abs(HH_plus*PP_plus-diag(diag(ones(size(HH_plus)))))+eps);
  emeas = log10(abs(HH_plus*PP_plus));
  imagesc(emeas); 
  axis equal tight; 
  colormap gray;
  colorbar;
  drawnow;
  %pause;
  %keyboard;
end
