function [u_minus,P_minus] = motionpredict_covform(u_plus,P_plus,xi,fu,F,Q)
%INIT_DSTATE_INFOFORM initialize a delayed state into Gaussian information form.
%   [n_minus,H_minus] = INIT_DSTATE_INFOFORM(b_plus,H_plus,xi,u_plus,fu,F,Q)
%   modifies the information vector, b_plus, and information matrix, H_plus,
%   to reflect augmentation of a delayed state to the end of the state
%   vector.  xi is the index of robot elements, fu is the nonlinear process
%   model evalulated at ux (i.e. the robot mean), F is the process model
%   Jacobian evaluated at ux, and Q is the process model noise.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-08-2004      rme         Created and written.

% extract block elements
Nx = length(xi);
Ny = length(u_plus)-Nx;
Pxx = P_plus(xi,xi);
Pyx = P_plus(Nx+1:end,xi);
Pyy = P_plus(Nx+1:end,Nx+1:end);

% augmented information vector
u_minus = [fu; ...
	   u_plus(Nx+1:end)];

% augmented information matrix
FPxy = F*Pyx';
P_minus = [ symmetrize(F*Pxx*F'+Q), FPxy; ...
	    FPxy',       Pyy ];

