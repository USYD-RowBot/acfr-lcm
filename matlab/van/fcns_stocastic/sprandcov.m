function P = sprandcov(n,d,k)
%SPRANDCOV generates a valid random covariance matrix.
%   P = SPRANDCOV(N,density) generates a sparse random symmetric positive
%   definite [N x N] covariance matrix via the P = L*D*L' decomposition
%   where L is a unit diagonal lower triangular matrix and D is a diagonal
%   matrix with positive pivots.  This decomposition guarantees the
%   resulting P is a symmetric positive definite matrix and hence a valid
%   covariance matrix.
%
%   P = SPRANDCOV(N,density,K) generates a sparse random positive
%   semi-definite [N x N] covariance marix of rank K.  This is accomplished
%   by setting (N-K) of the diagonal pivots to zero.
%
%   Note: The resulting covariance matrix can be scaled to have any desired
%   sigma terms on the main diagonal by computing the correlation
%   coefficient matrix and then mapping back with the desired standard
%   deviations.
%   Example:
%   P  = sprandcov(5,0.5);  % 5 x 5 positive definite sparse covariance matrix
%   R  = rhomatrix(P);      % corresponding correlation coefficient matrix
%   S  = diag([1 2 3 4 5]); % desired standard deviations
%   PP = S*R*S;             % scaled positive definite covariance matrix 
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-17-2004      rme         Created and written.
%    07-08-2004      rme         Cleaned up code to have less overhead.

if ~exist('d','var') || isempty(d)
  d = 0.5;
end
  
if ~exist('k','var') || isempty(k)
  k = n;
end

% generate a sparse random unit diagonal lower triangular matrix
L = tril(sprandn(n,n,d));
L(sub2ind([n,n],1:n,1:n)) = 1;

% generate a diagonal matrix with k non-zero positive pivots
D = spdiags([rand(k,1); spalloc(n-k,1,0)], 0, n, n);

% compute the resulting valid covariance matrix
P = L*D*L';

% symmetrize
P = (P + P')/2;

