function [mu_y,P_yy,P_xy] = sigmaTransfer(fhandle,mu_x,P_xx,h,varargin);
%function [mu_y,P_yy,P_xy] = sigmaTransfer(fhandle,mu_x,P_xx,h,varargin);  
%
%    Numerically computes covariance based upon sampling sigma points.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-28-2004      rme         Created and written.
%    12-22-2004      rme         Renamed from cdkf.m to sigmaTransfer.m
  
if ~exist('h','var') || isempty(h);
  h = sqrt(3); % optimal for a Gaussian prior
end;

L = length(mu_x);
r = 2*L+1;

% cholesky factorization
R_xx = chol(h^2*P_xx);

% sigma points
X = repmat(mu_x,[1 r]) + [zeros(L,1), R_xx, -R_xx];

% weights
w = [(1-L/h^2), repmat(1/(2*h^2),[1,2*L])];

% transform 0th sigma point to determine dimension of output space
Y0 = feval(fhandle,X(:,1),varargin{:});
M  = length(Y0);

% transform the remaining sigma points
Y  = zeros(M,r);
Y(:,1) = Y0;
mu_y = w(1)*Y0;
for ii=2:r;
  % evaluate model at sigma point
  Y(:,ii) = feval(fhandle,X(:,ii),varargin{:});
  % update weighted mean
  mu_y = mu_y + w(ii)*Y(:,ii);
end;

% compute transformed covariance and cross-covariance
P_yy = zeros(M);
P_xy = zeros(L,M);
for ii=1:r;
  X_tilde = X(:,ii) - mu_x;
  Y_tilde = Y(:,ii) - mu_y;
  P_yy = P_yy + w(ii)*Y_tilde*Y_tilde';
  P_xy = P_xy + w(ii)*X_tilde*Y_tilde';
end;
