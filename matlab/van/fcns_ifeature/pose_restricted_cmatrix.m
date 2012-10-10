function cmatrix = pose_restricted_cmatrix(u2,v2,u2p,v2p,Cov_u2pv2p,chiSquare2dof);
%function cmatrix = pose_restricted_cmatrix(u2,v2,u2p,v2p,Cov_u2pv2p,chiSquare2dof);
%  
% Points (u1,v1) are mapped to points (u2p,v2p) by the two-view point
% transfer function.  To 1st order, Cov_u2pv2p, is the covariance of
% the random vector [u2p,v2p]'.  We can use the statistics on
% [u2p,v2p]' to determine a bounding search region to establish
% possible correspondences between (u1,v1) and points in the set
% (u2,v2).  The bounding search region is evalated assumming a
% Gaussian distribution on [u2p,v2p]' and chiSquare2dof defines the perimeter
% of the bounding ellipse.  chiSquare2dof is a chi-square random variable
% with 2 degrees of freedom, i.e. chiSquare2dof = chi2inv(alpha,2) where
% alpha is in the range [0,1].
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-31-2003      rme         Created from feature_navcorr.m
%    04-14-2004      rme         Modified cmatrix test to be more matlab efficient.
%    11-04-2004      rme         Added variable persistence to be more efficient.
%    12-22-2004      rme         Changed Cov_u2pv2p from a cell to a [2 x 2 x N] array.
%    12-25-2004      rme         Modified to take chiSquare2dof as an input argument instead
%                                of confidence level alpha.  This makes it easier to
%                                scale the bounding ellipse size across multiple functions.

error(nargchk(6,6,nargin));

M = length(u2p);
N = length(u2);
cmatrix = false(M,N);
for ii=1:M;
  u_error = u2-u2p(ii);
  v_error = v2-v2p(ii);
  Sigma   = squeeze(Cov_u2pv2p(:,:,ii));
  Lambda  = Sigma^-1;
  % bounding ellipse is given by:
  % (u-u2p)'*Lambda*(u-u2p) = chiSquare2dof
  epsilon =     Lambda(1,1)*u_error.*u_error ...
	    + 2*Lambda(1,2)*u_error.*v_error ...
	    +   Lambda(2,2)*v_error.*v_error;
  cmatrix(ii,epsilon <= chiSquare2dof) = true;
end;
