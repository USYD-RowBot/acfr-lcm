function d = mahalanobis_dist(x,mu,Sigma)
%MAHALANOBIS_DIST Mahalanobis distance.
%   D = MAHALANOBIS_DIST(X,MU,SIGMA) returns the Mahalanobis distance D between the
%   unknown sample X and the known sample mean MU and covariance SIGMA.
%   D = [(X-MU)'*inv(SIGMA)*(X-MU)]^(1/2)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-16-2003      rme         Created and written.
%    09-27-2003      rme         Added minimum tolerance check.
%    11-08-2004      rme         Switch over to mu,Sigma notation.

if issparse(Sigma);
  Sigma = full(Sigma);
end

tol = max(size(Sigma))*norm(Sigma)*eps;   % calculate default matlab PINV tolerance
tol = max(1e-10,tol);                     % prescribe a minimum tolerance
d = sqrt((x-mu)'*pinv(Sigma,tol)*(x-mu)); % compute mahalanobis distance
