function [eta_x_given_y,Lambda_x_given_y] = condgauss_info(eta,Lambda,yi,y)
%CONDGAUSS_INFO computes statistics of Gaussian conditional pdf.
%   [eta_x_given_y,Lambda_x_given_y] = CONDGAUSS_INFO(eta,Lambda,yi,y)
%   computes the conditional information form of the jointly-Gaussian
%   random variable Z over the elements specified in the index vector yi
%   using the specified information vector eta, information matrix Lambda,
%   at the specified value y.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.
%    07-09-2004      rme         Modified to be more efficient.
%    08-20-2004      rme         Changed b to be a column vector
%    10-14-2004      rme         Switched to eta, Lambda notation
  
% The random vector z is composed of:
% z = [x', y']'
% Hz = [Hxx Hxy;
%       Hyx Hyy]
% and has a Gaussian distributed pdf of the form:
% p(z) = N^-1(bz,Hz)
%
% The conditional pdf of x given y is given by:
% i.e.  p(x|y) = p(x,y)/p(y)

% The argument yi contains the indicies we wish to condition over

% Generate the X element indicies
Nz = length(bz);
xi = 1:Nz;
xi(yi) = [];

% Compute some partitioned terms
Lambda_xx = Lambda(xi,xi);
Lambda_xy = Lambda(xi,yi);
eta_x  = eta(xi);

% Compute the conditional statistics for a given y
eta_x_given_y = eta_x - Lambda_xy*y;
Lambda_x_given_y = Lambda_xx;
