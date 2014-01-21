function [zpredict,z_fix,R_fix,Hv] = om_rdi_uvwrp(Xv,index_t,z_raw,R_raw,Rvs,x_vs)
%INPUTS:  see om_rdi_uvwrph.m
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-28-2003      rme         Created and written.
%    04-12-2004      rme         Updated to use index_t structure.
%    04-23-2004      rme         Reorganized to keep code maintenance simpler.
%    11-26-2004      rme         Reordered output args to make Hv last.
  
% evaluate the full sensor observation model
[zpredict,z_fix,R_fix,Hv] = om_rdi_uvwrph(Xv,index_t,z_raw,R_raw,Rvs,x_vs);

% only return the prediction associated with sensor frame velocities
% and roll,pitch
zpredict = zpredict(1:5);
z_fix    = z_fix(1:5);
R_fix    = R_fix(1:5,1:5);
Hv       = Hv(1:5,:);
