function [n_minus,H_minus] = motionpredict_infoform(n_plus,H_plus,xi,u_plus,fu,F,Q)
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
Ny = length(n_plus)-Nx;
Hxx = H_plus(xi,xi);
Hyx = H_plus(Nx+1:end,xi);
Hyy = H_plus(Nx+1:end,Nx+1:end);

% useful terms to precompute
W = (fu - F*u_plus(xi));
Qinv  = symmetrize(Q^-1);
Phi   = symmetrize( (Q + F*Hxx^-1*F')^-1 );
Omegainv = symmetrize( (Hxx + F'*Qinv*F)^-1 );

% augmented information vector
n_minus = [Qinv*F*Omegainv*n_plus(xi) + Phi*W; ...
	   n_plus(Nx+1:end) - Hyx*Omegainv *(n_plus(xi) - F'*Qinv*W) ];

% augmented information matrix
L = Qinv*F*Omegainv*Hyx';
H_minus = [ Phi,                   L; ...
	    L',   Hyy - Hyx*Omegainv*Hyx' ];

