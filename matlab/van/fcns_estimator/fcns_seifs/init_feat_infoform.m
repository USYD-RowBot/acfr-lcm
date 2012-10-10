function [b_minus,H_minus] = init_feat_infoform(b_plus,H_plus,xi,guz,Gx,Gz,R)
%INIT_FEAT_INFOFORM initialize a feature into Gaussian information form.
%   [b_minus,H_minus] = INIT_FEAT_INFOFORM(b_plus,H_plus,xi,guz,Gx,Gz,R)
%   modifies the information vector, b_plus, and information matrix, H_plus,
%   to reflect augmentation of a feature to the end of the state vector.  xi
%   is the index of robot elements, guz is the nonlinear feature
%   initalization model evalulated at u_plus and z, Gx and Gz are the
%   Jacobians evaluated at u_plus and z respectively, and R is the
%   measurement covariance.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-24-2004      rme         Created and written.

% extract block elements
Nx = length(xi);
Ny = length(b_plus)-Nx;
Hxx = H(xi,xi);
Hyy = H(Nx+1:end,Nx+1:end);
Hyx = H(Nx+1:end,xi);

% useful terms to precomute
Z = symmetrize( (Gz*R*Gz')^-1 ); % force numerical symmetry
W = (guz - Gx*u_plus(xi))'*Z;

% augmented information vector
b_minus = [(b_plus(xi) - W*Gx), b_plus(Nx+1:end), W]; 

% augmented information matrix
H_minus = [(Hxx + Gx'*Z*Gx),        Hyx',               -Gx'*Z; ...
	    Hyx,                    Hyy,           spalloc(Ny,Nx,0); ...
	   -Z*Gx,             spalloc(Nx,Ny,0),          Z];

