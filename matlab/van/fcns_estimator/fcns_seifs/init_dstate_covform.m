function [u_minus,P_minus] = init_dstate_covform(u_plus,P_plus,xi,fu,F,Q)
%INIT_DSTATE_COVFORM initialize a delayed state into Gaussian covariance form.
%   [u_minus,P_minus] = INIT_DSTATE_COVFORM(u_plus,P_plus,xi,fu,F,Q)
%   modifies the mean, u_plus, and covariance matrix, P_plus, to reflect
%   augmentation of a delayed state to the end of the state vector.  xi is
%   the index of robot elements, fu is the nonlinear process model
%   evalulated at u_plus, F is the process model Jacobian evaluated at
%   u_plus, and Q is the process model noise.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-24-2004      rme         Created and written.
%    09-06-2004      rme         Reordered.

% extract block elements
Nx = length(xi);
Pxx = P_plus(xi,xi);
Pyy = P_plus(Nx+1:end,Nx+1:end);
Pyx = P_plus(Nx+1:end,xi);

FPxxF = symmetrize(F*Pxx*F');

% augmented mean vector
u_minus = [fu; ...
	   u_plus(xi); ...
	   u_plus(Nx+1:end) ];

% augmented covariance matrix
P_minus = [(FPxxF + Q),   F*Pxx,  F*Pyx'; ...
	     Pxx*F',        Pxx,    Pyx', 
             Pyx*F',        Pyx,    Pyy ];
