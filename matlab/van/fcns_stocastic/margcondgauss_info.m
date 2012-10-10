function [eta_prime,Lambda_prime] = margcondgauss_info(eta,Lambda,ai,bi,ci,c)
%MARGCONDGAUSS_INFO computes statistics of marginalized and conditioned Gaussian pdf.
%   [eta_prime,Lambda_prime] = MARGCONDGAUSS_INFO(eta,Lambda,ai,bi,ci,c) conditions
%   and marginalizes the information form of the jointly-Gaussian random
%   variable Z.  Assuming Z is defined by a Gaussian distribution p(a,b,c),
%   the distribution p(a | c) is returned.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    08-17-2004      rme         Updated.
%    08-20-2004      rme         Changed b to be a column vector
%    10-14-2004      rme         Switched to eta, Lambda notation
%    11-01-2004      rme         Modified to use spdproduct.m & spdinverse.m  
  
% The argument bi contains the indicies we wish to marginalize p(z) over
% while ci contains the indicies we wish to condition on.

% Compute some partitioned terms
Lambda_aa = Lambda(ai,ai);
Lambda_ab = Lambda(ai,bi);
Lambda_ac = Lambda(ai,ci);
Lambda_bc = Lambda(bi,ci);
invLambda_bb = spdinverse(Lambda(bi,bi));
eta_a = eta(ai);
eta_b = eta(bi);

% compute the parameters of the desired distribution
eta_prime = (eta_a - Lambda_ac*c) - Lambda_ab*invLambda_bb*(eta_b - Lambda_bc*c);
Lambda_prime = Lambda_aa - spdproduct(invLambda_bb,Lambda_ab');
