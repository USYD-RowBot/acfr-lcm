function [zpredict,z_fix,R_fix,Hv] = om_phins_h(Xv,index_t,z_raw,R_raw,x_vs)
%INPUTS: see om_phins_rph.m
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    02-27-2004      rme         Created from om_xbow.m
%    03-29-2004      rme         Incorporated nonlinear observation model.
%    04-12-2004      rme         Updated to use index_t structure.  
%    04-23-2004      rme         Reorganized to keep code maintenance simpler.
%    11-26-2004      rme         Reordered output args to make Hv last.
  
% evaluate the full sensor observation model
[zpredict,z_fix,R_fix,Hv] = om_phins_rph(Xv,index_t,z_raw,R_raw,x_vs);

% only return the prediction associated with sensor frame heading
zpredict = zpredict(3);
z_fix    = z_fix(3);
R_fix    = R_fix(3,3);
Hv       = Hv(3,:);
