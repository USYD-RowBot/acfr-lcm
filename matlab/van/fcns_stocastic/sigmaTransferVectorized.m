function [mu_y,P_yy,P_xy] = sigmaTransferVectorized(fhandle,mu_x,P_xx,h,varargin);
%function [mu_y,P_yy,P_xy] = sigmaTransfer(fhandle,mu_x,P_xx,h,varargin);  
%
%    Numerically computes covariance based upon sampling sigma points.
%
% fhandle: function handle
%    mu_x: [L x 1] vector
%    P_xx: [L x L] matrix
%       h: sample parameter
%varargin: optioinal fhandle params
%
%      fhandle is assumed to return a vectorized output Y = [M x N]
%      in which case the following statistics are computed
%    mu_y: [M x N];
%    P_yy: [M x M x N];
%    P_xy: [L x M x N];
%
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-22-2004      rme         Created and written form sigmaTransfer.m
  
if ~exist('h','var') || isempty(h);
  h = sqrt(3); % optimal for a Gaussian prior
end;

L = length(mu_x);
r = 2*L+1;

% cholesky factorization
try
  R_xx = chol(h^2*P_xx);
catch;
  warning('P_xx not Pos Def!');
  keyboard;
end;

% sigma points
X = repmat(mu_x,[1 r]) + [zeros(L,1), R_xx, -R_xx];

% weights
w = [(1-L/h^2), repmat(1/(2*h^2),[1,2*L])];

% transform 0th sigma point to determine dimension of output space
Y0 = feval(fhandle,X(:,1),varargin{:});
[M,N] = size(Y0);

% transform the remaining sigma points
Y  = zeros(M,r,N);
Y(:,1,:) = Y0;
mu_y = w(1)*Y(:,1,:);
for ii=2:r;
  % evaluate model at sigma point
  Y(:,ii,:) = feval(fhandle,X(:,ii),varargin{:});
  % update weighted mean
  mu_y = mu_y + w(ii)*Y(:,ii,:);
end;

% compute transformed covariance and cross-covariance
P_yy = zeros(M,M,N);
if nargout == 3;
  P_xy = zeros(L,M,N);
end;
for ii=1:r;
  Y_tilde = Y(:,ii,:) - mu_y;
  P_yy = P_yy + w(ii)*outerproduct(Y_tilde,Y_tilde);
  if nargout == 3;
    X_tilde = repmat(X(:,ii) - mu_x, [1,1,N]);
    P_xy = P_xy + w(ii)*outerproduct(X_tilde,Y_tilde);
  end;
end;

% convert mu_y from a [M x 1 x N] to a [M x N] matrix
mu_y = squeeze(mu_y);


%===============================================================================
function Z = outerproduct(X,Y)
% X is a [L x 1 x N] matrix
% Y is a [M x 1 x N] matrix
%
% Z is a [L x M x N] matrix

[L,junk,N] = size(X);
[M,junk,N] = size(Y);

% reshape Y into a [1 x M x N], i.e. transpose for each Nth level
Y = reshape(Y,[1,M,N]);

% outer product for each Nth level
Z = repmat(X,[1,M,1]) .* repmat(Y,[L,1,1]);
