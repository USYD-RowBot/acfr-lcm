function [eta_prime,Lambda_prime] = marggauss_info(eta,Lambda,yi)
%MARGGAUSS_INFO computes statistics of Gaussian marginal pdf.
%   [eta_x,Lambda_xx] = MARGGAUSS_INFO(eta,Lambda,yi) marginalizes the
%   information form of the jointly-Gaussian random variable Z over
%   the elements specified in the index vector yi using the specified
%   information vector eta and information matrix Lambda.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    08-20-2004      rme         Changed b to be a column vector
%    10-14-2004      rme         Switched to eta, Lambda notation
%    11-01-2004      rme         Modified to use spdproduct.m & spdinverse.m
  
% The random vector z is composed of:
% z = [x', y']'
% Hz = [Hxx Hxy;
%       Hyx Hyy]
% and has a Gaussian distributed pdf of the form:
% p(z) = N^-1(bz,Hz)
%
% The marginal over x is given by:
% i.e.  p(x) = integral p(x,y) dy
%                 y

% The argument yi contains the indicies we wish to marginalize p(z) over

% Generate the X element indicies
Nz = length(eta);
xi = 1:Nz;
xi(yi) = [];

% Compute some partitioned terms
Lambda_xx = Lambda(xi,xi);
Lambda_xy = Lambda(xi,yi);
invLambda_yy = spdinverse(Lambda(yi,yi));
eta_x = eta(xi);
eta_y = eta(yi);

% Marginalize the statistics
eta_prime = eta_x  - Lambda_xy*invLambda_yy*eta_y;
Lambda_prime = Lambda_xx - spdproduct(invLambda_yy,Lambda_xy');

