function [u_minus,P_minus] = init_feat_covform(u_plus,P_plus,xi,guz,Gx,Gz,R)
%INIT_FEAT_COVFORM initialize a feature into Gaussian covariance form.
%   [u_minus,P_minus] = INIT_DFEAT_COVFORM(u_plus,P_plus,xi,guz,Gx,Gz,R)
%   modifies the mean, u_plus, and covariance matrix, P_plus, to reflect
%   augmentation of a feature to the end of the state vector.  xi is the
%   index of robot elements, guz is the nonlinear feature initalization
%   model evalulated at u_plus and z, Gx and Gz are the Jacobians evaluated
%   at u_plus and z respectively, and R is the measurement covariance.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-24-2004      rme         Created and written.

% extract block elements
Nx = length(xi);
Pxx = P_plus(xi,xi);
Pyy = P_plus(Nx+1:end,Nx+1:end);
Pyx = P_plus(Nx+1:end,xi);

% augmented mean vector
u_minus = [u_plus; guz];

% augmented covariance matrix
P_minus = [   Pxx,     Pyx',     Pxx*Gx'; ...
	      Pyx,     Pyy,      Pyx*Gx'; ...
	   Gx*Pxx,  Gx*Pyx', (Gx*Pxx*Gx' + Gz*R*Gz')];

