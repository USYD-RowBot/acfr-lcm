function [zpredict,z_fix,R_fix,Hv] = om_rdi_rph(Xv,index_t,z_raw,R_raw,Rvs,x_vs)
%INPUTS:  see om_rdi_uvwrph.m
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-10-2004      rme         Created from om_rdi_uvw.m
%    11-26-2004      rme         Reordered output args to make Hv last.  

% evaluate the full sensor observation model
[zpredict,Hv,z_fix,R_fix] = om_rdi_uvwrph(Xv,index_t,z_raw,R_raw,Rvs,x_vs);

% only return the prediction associated with attitude
zpredict = zpredict(4:6);
Hv       = Hv(4:6,:);
z_fix    = z_fix(4:6);
R_fix    = R_fix(4:6,4:6);
