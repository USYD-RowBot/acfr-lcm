clear all;

xi = 1:12;
mi = 13:24;

Nx = length(xi);
Nm = length(mi);

% process model
switch 'test'
case 'test'
 F = 1e-3*randn(Nx);
 B = randn(Nx);
 u = randn(Nx,1);
 Q = 1e-6*randcov(Nx);
case 'fake'
  load fakeProcessModel;
  F = Fk;
  B = Bk;
  u = uk;
  Q = Qk;
otherwise 
 disp('no model selected');
end
Qinv = spdinverse(Q);

% covariance
SigmaAug  = 1e-6*randcov(Nx+Nm);
muAug     = randn(Nx+Nm,1);
LambdaAug = spdinverse(SigmaAug);
etaAug    = LambdaAug*muAug;

yi = xi+Nx+Nm;
for ii=1:1000

  % pointers
  mu_x     = muAug(xi);
  mu_M     = muAug(mi);
  Sigma_xx = SigmaAug(xi,xi);
  Sigma_xM = SigmaAug(xi,mi);

  eta_x     = etaAug(xi);
  eta_M     = etaAug(mi);
  Lambda_xx = LambdaAug(xi,xi);
  Lambda_xM = LambdaAug(xi,mi);

  %u = muAug(xi);
  %u_ekf = u;
  % augment covariance form
  SigmaAug(yi,yi) = Q + spdproduct(Sigma_xx,F');
  SigmaAug(yi,xi) = F*Sigma_xx;
  SigmaAug(xi,yi) = SigmaAug(yi,xi)';
  SigmaAug(yi,mi) = F*Sigma_xM;
  SigmaAug(mi,yi) = SigmaAug(yi,mi)';
  
  muAug(yi) = F*mu_x + B*u;

  %tmp = LambdaAug \ etaAug;
  %u = tmp(xi);
  %u_eif = u;
  % augment information form
  LambdaAug(yi,yi) =  Qinv;
  LambdaAug(yi,xi) = -Qinv*F;
  LambdaAug(xi,yi) =  LambdaAug(yi,xi)';
  LambdaAug(xi,xi) =  Lambda_xx + spdproduct(Qinv,F);
  
  etaAug(yi) = Qinv*B*u;
  etaAug(xi) = eta_x - F'*etaAug(yi);
  
  %max(abs(u_eif-u_ekf))
  %keyboard;
  
  kth = 100;
  istrueAugment = (mod(ii,kth) == 0);
  if istrueAugment
    % do nothing
    fprintf('Augmenting ... size(LambdaAug)=[%d %d]\n',length(etaAug),length(etaAug));
    xi = yi;
    mi = 1:xi(1)-1;
    yi = yi+Nx;
  else
    % prediction step
    [muAug,SigmaAug] = marggauss_cov(muAug,SigmaAug,xi);
    [etaAug,LambdaAug] = marggauss_info(etaAug,LambdaAug,xi);
  end
  
end
