function [mu_prime,Sigma_prime] = margcondgauss_cov(mu,Sigma,ai,bi,ci,c)
%MARGCONDGAUSS_COV computes statistics of marginalized and conditioned Gaussian pdf.
%   [mu_prime,Sigma_prime] = MARGCONDGAUSS_COV(mu,Sigma,ai,bi,ci,c)
%   conditions and marginalizes the covariance form of the jointly-Gaussian
%   random variable Z.  Assuming Z is defined by a Gaussian distribution
%   p(a,b,c), the distribution p(a | c) is returned.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    08-17-2004      rme         Updated.
%    10-14-2004      rme         Switched to mu, Sigma notation
%    11-01-2004      rme         Modified to use spdproduct.m & spdinverse.m

% The argument bi contains the indicies we wish to marginalize p(z) over
% while ci contains the indicies we wish to condition on.

% Compute some partitioned terms
Sigma_aa = Sigma(ai,ai);
Sigma_ac = Sigma(ai,ci);
invSigma_cc = spdinverse(Sigma(ci,ci));
mu_a = mu(ai);
mu_c = mu(ci);

% compute the parameters of the desired distribution
mu_prime = mu_a + Sigma_ac*invSigma_cc*(c - mu_c);
Sigma_prime = Sigma_aa - spdproduct(invSigma_cc,Sigma_ac');
