function [Xdot,Fv] = pm_constant_velocity(X,U,index_t)
%INPUTS
% vehicle state vector is assumed to contain the following elements
% not necessarily in the order shown
% Xv = [x u  y v  z w  r a  p b  h c]'
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
%    07-21-2003      rme         Created & written.
%    08-18-2003      rme         Added Jacobian.
%    08-19-2003      rme         Fixed to accept any state element paritioning.
%    09-28-2003      rme         Dropped X_dtype code baggage.
%    04-10-2004      rme         Precalc rotation matrices in Jacobian calculation
%                                Changed abc to represent *body* frame angular rates
%    04-10-2004      rme         added index_t argument

% define shorthand index for state vector elements
xyz_i = index_t.xyz_i;  uvw_i = index_t.uvw_i;
rph_i = index_t.rph_i;  abc_i = index_t.abc_i;

% get values from state vector
xyz     = X(xyz_i);     %local-level position
uvw     = X(uvw_i);     %body frame velocities
rph     = X(rph_i);     %local-level Euler angles
abc     = X(abc_i);     %body frame angular rates

% rotation matrix vehicle to local-level
Rlv = rotxyz(rph);

% vehicle velocity in local-level frame
xyz_dot = Rlv*uvw;

% Euler angular rates in local-level frame
[rph_dot,Jabc_rph] = body2euler(abc,rph);

% constant velocity process model
Xdot        = zeros(index_t.Nv,1);
Xdot(xyz_i) = xyz_dot;
Xdot(uvw_i) = [0,0,0]';
Xdot(rph_i) = rph_dot;
Xdot(abc_i) = [0,0,0]';

% calculate the Jacobian
if nargout == 2
  ROTX = rotx(rph(1)); DROTX = drotx(rph(1));
  ROTY = roty(rph(2)); DROTY = droty(rph(2));
  ROTZ = rotz(rph(3)); DROTZ = drotz(rph(3));
  
  % allocate sparse matrix
  Fv = spalloc(index_t.Nv,index_t.Nv,36);

  % fill in non-zero entries for xyz_dot
  Fv(xyz_i,uvw_i)    = Rlv;                     % deriv xyz_dot w.r.t. uvw
  Fv(xyz_i,rph_i(1)) = ROTZ'*ROTY'*DROTX'*uvw;  % deriv xyz_dot w.r.t. r
  Fv(xyz_i,rph_i(2)) = ROTZ'*DROTY'*ROTX'*uvw;  % deriv xyz_dot w.r.t. p
  Fv(xyz_i,rph_i(3)) = DROTZ'*ROTY'*ROTX'*uvw;  % deriv xyz_dot w.r.t. h

  % fill in non-zero entries for rph_dot
  Fv(rph_i,rph_i) = Jabc_rph(:,4:6);            % deriv rph_dot w.r.t. rph
  Fv(rph_i,abc_i) = Jabc_rph(:,1:3);            % deriv rph_dot w.r.t. abc
end
