function [Xdot,Fv] = pm_constant_acceleration(X,U,index_t)
%INPUTS
% vehicle state vector is assumed to contain the following elements
% not necessarily in the order shown  
% Xv = [x u udot  y v vdot  z w wdot  r a adot  p b bdot  h c cdot]'
%
% control vector
% U = [empty]
%
%OUTPUTS
% Xdot
% Fv Jacobian w.r.t. vehicle state (optional)
%
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created & written.
  
% define shorthand index for state vector elements
xyz_i = index_t.xyz_i;  uvw_i = index_t.uvw_i;  uvw_dot_i = index_t.uvw_dot_i;
rph_i = index_t.rph_i;  abc_i = index_t.abc_i;  abc_dot_i = index_t.abc_dot_i;

% get values from state vector
xyz     = X(xyz_i);     %local-level position
uvw     = X(uvw_i);     %body frame velocities
uvw_dot = X(uvw_dot_i); %body frame accelerations
rph     = X(rph_i);     %local-level Euler angles
abc     = X(abc_i);     %body frame angular rates
abc_dot = X(abc_dot_i); %body frame angular accelerations

% rotation matrix vehicle to local-level
Rlv = rotxyz(rph);

% vehicle velocity in local-level frame
xyz_dot = Rlv*uvw;

% Euler angular rates in local-level frame
[rph_dot,Jabc_rph] = body2euler(abc,rph);

% constant acceleration process model
Xdot            = zeros(index_t.Nv,1);
Xdot(xyz_i)     = xyz_dot;
Xdot(uvw_i)     = uvw_dot;
Xdot(uvw_dot_i) = [0,0,0]';
Xdot(rph_i)     = rph_dot;
Xdot(abc_i)     = abc_dot;
Xdot(abc_dot_i) = [0 0 0]';

% calculate the Jacobian
if nargout == 2
  ROTX = rotx(rph(1)); DROTX = drotx(rph(1));
  ROTY = roty(rph(2)); DROTY = droty(rph(2));
  ROTZ = rotz(rph(3)); DROTZ = drotz(rph(3));
  
  % allocate sparse matrix
  Fv = spalloc(index_t.Nv,index_t.Nv,42);

  % fill in non-zero entries for xyz
  Fv(xyz_i,uvw_i)    = Rlv;                     % deriv xyz w.r.t. uvw
  Fv(xyz_i,rph_i(1)) = ROTZ'*ROTY'*DROTX'*uvw;  % deriv xyz w.r.t. r
  Fv(xyz_i,rph_i(2)) = ROTZ'*DROTY'*ROTX'*uvw;  % deriv xyz w.r.t. p
  Fv(xyz_i,rph_i(3)) = DROTZ'*ROTY'*ROTX'*uvw;  % deriv xyz w.r.t. h

  % fill in non-zero entries for uvw
  Fv(uvw_i,uvw_dot_i) = speye(3);               % deriv uvw w.r.t. uvw_dot
  
  % fill in non-zero entries for rph
  Fv(rph_i,rph_i) = Jabc_rph(:,4:6);            % deriv rph w.r.t. abc
  Fv(rph_i,abc_i) = Jabc_rph(:,1:3);            % deriv rph w.r.t. abc

  % fill in non-zero entries for abc
  Fv(abc_i,abc_dot_i) = speye(3);               % deriv abc w.r.t. abc_dot
end
