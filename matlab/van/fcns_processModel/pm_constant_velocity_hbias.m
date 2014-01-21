function [Xdot,Fv] = pm_constant_velocity_hbias(X,U)
%INPUTS
% vehicle state vector is assumed to contain the following elements
% X = [x u y v z w r rdot p pdot h  hdot bh]'
%      1 2 3 4 5 6 7 8    9 10   11 12   13 index
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
  
% define shorthand index for state vector elements
xi = 1; yi = 3; zi = 5;
ui = 2; vi = 4; wi = 6;
ri = 7; pi = 9; hi = 11;
rri = 8; pri = 10; hri = 12; bhi = 13;

xyz = X([xi;yi;zi]); %local-level position
rph = X([ri;pi;hi]); %local-level Euler angles
uvw = X([ui;vi;wi]); %body frame velocities
rph_dot = X([rri;pri;hri]); %local-level angular rates
bh  = X(bhi);        %heading angle bias

% rotation matrix vehicle to local-level
Rlv = rotxyz(rph);
% vehicle velocity in local-level frame
xyz_dot = Rlv*uvw;

Xdot = zeros(size(X));
Xdot([xi,yi,zi]) = xyz_dot;
Xdot([ri,pi,hi]) = rph_dot;
Xdot([ui,vi,wi]) = [0 0 0]';
Xdot([rri,pri,hri]) = [0 0 0]';
%Xdot(bhi) = 0;
% Gauss-Markov heading bias has the following differential equation:
% xdot(t) = -beta*x(t) + sqrt(2*beta*sigma^2)*w(t)
% where w(t) is unity white Gaussian noise
%       beta is one over the process-time constant
%       sigma is the process variance
beta = 1/6; % tau in seconds
Xdot(bhi) = -beta*bh;


if nargout == 2
  % calculate the Jacobian
  Fv = spalloc(size(X,1),size(X,1),21);
  Fv([xi,yi,zi],ri) = rotz(rph(3))'*roty(rph(2))'*drotx(rph(1))'*uvw; % deriv w.r.t. r
  Fv([xi,yi,zi],pi) = rotz(rph(3))'*droty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. p
  Fv([xi,yi,zi],hi) = drotz(rph(3))'*roty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. h
  Fv([xi,yi,zi],[ui,vi,wi]) = Rlv;         % deriv w.r.t. uvw
  Fv([ri,pi,hi],[rri,pri,hri]) = speye(3); % deriv w.r.t. rph_dot
  Fv(bhi,bhi) = -beta; % deriv w.r.t. bh
end
