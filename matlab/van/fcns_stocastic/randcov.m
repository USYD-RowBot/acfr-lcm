function P = randcov(n,k)
%RANDCOV generates a valid random covariance matrix.
%   P = RANDCOV(N) generates a random symmetric positive definite [N x N]
%   covariance matrix via P = L'*L where L is a random [N x N] matrix.
%   This decomposition guarantees the resulting P is symmetric positive
%   definite matrix and hence a valid covariance matrix.
%
%   P = RANDCOV(N,K) generates a random positive semi-definite [N x N]
%   covariance marix of rank K.  This is accomplished by setting P = L'*D*L
%   where D is an identity matrix with (N-K) of the diagonal elements set
%   to zero.
%
%   Note: The resulting covariance matrix can be scaled to have any desired
%   sigma terms on the main diagonal by computing the correlation
%   coefficient matrix and then mapping back with the desired standard
%   deviations.
%   Example:
%   P  = randcov(5);        % 5 x 5 positive definite covariance matrix
%   R  = rhomatrix(P);      % corresponding correlation coefficient matrix
%   S  = diag([1 2 3 4 5]); % desired standard deviations
%   PP = S*R*S;             % scaled positive definite covariance matrix 
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-15-2004      rme         Created and written.
%    10-16-2004      rme         Updated covariance matrix algorithm
%                                to be more numerically robust to rank deficiency.
%    05-16-2006      rme         Replaced call to speye() in calculation of D with
%                                call to eye().

if ~exist('k','var') || isempty(k)
  k = n;
end

% generate a random [n x n] matrix
L = randn(n);

if k < n
  % generate a diagonal matrix with k non-zero positive pivots
  D = eye(k);
  D(n,n) = 0;
else
  D = eye(n);
end

% compute the resulting valid covariance matrix
P = L'*D*L;

