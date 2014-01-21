function [zpredict,z_fix,R_fix,Hv] = om_xbow_c(Xv,index_t,z_raw,R_raw,Rvs)
%INPUTS: see om_xbow_abc.m
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-28-2003      rme         Created and written.
%    03-29-2004      rme         Modified observation model to predict
%                                sensor measurement instead of putting
%                                sessor in vehicle frame.
%    04-12-2004      rme         Updated to use index_t structure.
%    04-23-2004      rme         Reorganized to keep code maintenance simpler.
%    11-26-2004      rme         Reordered output args to make Hv last.

% evaluate the full sensor observation model
[zpredict,z_fix,R_fix,Hv] = om_xbow_abc(Xv,index_t,z_raw,R_raw,Rvs);

% only return the prediction associated with sensor frame heading rate
zpredict = zpredict(3);
z_fix    = z_fix(3);
R_fix    = R_fix(3,3);
Hv       = Hv(3,:);
