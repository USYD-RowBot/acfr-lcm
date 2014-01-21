function [zpredict,z_fix,R_fix,Hv] = om_octans_h(Xv,index_t,z_raw,R_raw,x_vs)
%INPUTS: see om_octans_rph.m
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-02-2004      rme         Created from om_phins_h.m
%    11-26-2004      rme         Reordered output args to make Hv last.
  
% evaluate the full sensor observation model
[zpredict,z_fix,R_fix,Hv] = om_octans_rph(Xv,index_t,z_raw,R_raw,x_vs);

% only return the prediction associated with sensor frame heading
zpredict = zpredict(3);
z_fix    = z_fix(3);
R_fix    = R_fix(3,3);
Hv       = Hv(3,:);
