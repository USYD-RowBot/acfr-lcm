function [mu_x_given_y,Sigma_x_given_y] = condgauss_cov(mu,Sigma,yi,y)
%CONDGAUSS_COV computes statistics of Gaussian conditional pdf.
%   [mu_x_given_y,Sigma_x_given_y] = CONDGAUSS_COV(mu,Sigma,yi,y) computes
%   the conditional statistics of the jointly-Gaussian random variable Z
%   over the elements specified in the index vector yi using the specified
%   mean mu, covariance matrix Sigma, at the specified value y.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    10-14-2004      rme         Switched to mu, Sigma notation
%    11-01-2004      rme         Modified to use spdproduct.m & spdinverse.m

% The random vector z is composed of:
% z = [x', y']'
% Pz = [Pxx Pxy;
%       Pyx Pyy]
% and has a Gaussian distributed pdf of the form:
% p(z) = N(uz,Pz)
%
% The conditional pdf of x given y is given by:
% i.e.  p(x|y) = p(x,y)/p(y)

% The argument yi contains the indicies we wish to condition over

% Generate the X element indicies
Nz = length(uz);
xi = 1:Nz;
xi(yi) = [];

% Compute some partitioned terms
Sigma_xx = Sigma(xi,xi);
Sigma_xy = Sigma(xi,yi);
invSigma_yy = spdinverse(Sigma(yi,yi));
mu_x = mu(xi);
mu_y = mu(yi);

% Compute the conditional statistics for a given y
mu_x_given_y = mu_x + Sigma_xy*invSigma_yy*(y - mu_y);
Sigma_x_given_y = Sigma_xx - spdproduct(invSigma_yy,Sigma_xy');
