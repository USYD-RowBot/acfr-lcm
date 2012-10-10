function [n_minus,H_minus] = init_dstate_infoform(n_plus,H_plus,xi,u_plus,fu,F,Q)
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
%    06-24-2004      rme         Created and written.
%    09-06-2004      rme         Reordered.

% extract block elements
Nx = length(xi);
Ny = length(n_plus)-Nx;
Hxx = H_plus(xi,xi);
Hyx = H_plus(Nx+1:end,xi);
Hyy = H_plus(Nx+1:end,Nx+1:end);

% useful terms to precompute
Qinv = symmetrize(Q^-1);        % force numerical symmetry
FQinvF = symmetrize(F'*Qinv*F); % force numerical symmetry
W = Qinv*(fu - F*u_plus(xi));

% augmented information vector
n_minus = [W; ...
	   (n_plus(xi) - F'*W); ...
	   n_plus(Nx+1:end)];

% augmented information matrix
H_minus = [    Qinv,             -Qinv*F,       spalloc(Nx,Ny,0); ...
	   -F'*Qinv,          (Hxx + FQinvF),       Hyx'; ... 
	   spalloc(Ny,Nx,0)         Hyx,            Hyy  ];
