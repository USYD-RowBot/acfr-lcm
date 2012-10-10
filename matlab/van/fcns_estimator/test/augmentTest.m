clear all;

xi = 1:5;
mi = 6:15;

Nx = length(xi);
Nm = length(mi);

% process model
F = 1e-3*randn(Nx);
B = randn(Nx);
u = randn(Nx,1);
Q = 1e-6*randcov(Nx);
Qinv = spdinverse(Q);

% covariance
SigmaAug  = 1e-6*randcov(Nx+Nm);
muAug     = randn(Nx+Nm,1);
LambdaAug = spdinverse(SigmaAug);
etaAug    = LambdaAug*muAug;

for ii=1:100
  if ii == 1
    yi = xi+Nx+Nm;
  else
    xi = yi;
    mi = 1:xi(1)-1;
    yi = yi+Nx;
  end

  % pointers
  mu_x     = muAug(xi);
  mu_M     = muAug(mi);
  Sigma_xx = SigmaAug(xi,xi);
  Sigma_xM = SigmaAug(xi,mi);

  eta_x     = etaAug(xi);
  eta_M     = etaAug(mi);
  Lambda_xx = LambdaAug(xi,xi);
  Lambda_xM = LambdaAug(xi,mi);

  
  % augment covariance form
  SigmaAug(yi,yi) = Q + spdproduct(Sigma_xx,F');
  SigmaAug(yi,xi) = F*Sigma_xx;
  SigmaAug(xi,yi) = SigmaAug(yi,xi)';
  SigmaAug(yi,mi) = F*Sigma_xM;
  SigmaAug(mi,yi) = SigmaAug(yi,mi)';
  
  muAug(yi) = F*mu_x + B*u;

  % augment information form
  LambdaAug(yi,yi) =  Qinv;
  LambdaAug(yi,xi) = -Qinv*F;
  LambdaAug(xi,yi) =  LambdaAug(yi,xi)';
  LambdaAug(xi,xi) =  Lambda_xx + spdproduct(Qinv,F);
  
  etaAug(yi) = Qinv*B*u;
  etaAug(xi) = eta_x - F'*etaAug(yi);
  
end
