function [mu_x,Sigma_xx] = marggauss_cov(mu,Sigma,yi)
%MARGGAUSS_COV computes mean and covariance of Gaussian marginal pdf.
%   [mu_x,Sigma_xx] = MARGGAUSS_COV(mu,Sigma,yi) marginalizes the jointly-Gaussian
%   random variable Z over the elements specified in the index vector yi
%   using the specified mean uz and covariance Pz.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    10-14-2004      rme         Switched to mu, Sigma notation

% The random vector z is composed of:
% z = [x', y']'
% Pz = [Pxx Pxy;
%       Pyx Pyy]
% and has a Gaussian distributed pdf of the form:
% p(z) = N(zbar,Pz)
%
% The marginal over x is given by:
% i.e.  p(x) = integral p(x,y) dy
%                 y

% The argument yi contains the indicies we wish to marginalize p(z) over

% Generate the X element indicies
Nz = length(mu);
xi = 1:Nz;
xi(yi) = [];

% Marginalize the statistics
mu_x     = mu(xi);
Sigma_xx = Sigma(xi,xi);
