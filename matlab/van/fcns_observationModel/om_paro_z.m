function [zpredict,z_fix,R_fix,Hv] = om_paro_z(Xv,index_t,z_raw,R_raw,x_vs)
%INPUTS:
%  Xv is the vehicle state vector assumed to contain at 
%  least the following elements, (not necessarily in the order shown)
%  Xv = [x y z r p h]'
%  index_t is a structure of state indices
%  z_raw raw depth measurement of sensor
%  x_vs is a [6 x 1] vehicle to paro sensor coordinate transform
%
%  zpredict is the nonlinear predicted sensor measurement
%  Hv is the Jacobian matrix of the observation function
%  z_fix same as z_raw
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-28-2003      rme         Created and written.
%    03-22-2004      rme         Incorporated nonlinear coordinate frame
%                                transfrom to get predicted sensor measurement
%    04-12-2004      rme         Updated to use index_t structure
%    11-26-2004      rme         Reordered output args to make Hv last.

% pose variable index
Xp_i = index_t.Xp_i;

% vehicle pose in local-level frame
x_lv = Xv(Xp_i);

% predicted depth measurement in sensor frame
[x_ls,J_plus] = head2tail(x_lv,x_vs);

% predicted sensor measurement is the z-component of x_ls
zpredict = x_ls(3);

% jacobian w.r.t. Xv
Hv = spalloc(1,index_t.Nv,6);
Hv(1,Xp_i) = J_plus(3,1:6);

% no need to modify the raw depth measurement
z_fix = z_raw;
R_fix = R_raw;
