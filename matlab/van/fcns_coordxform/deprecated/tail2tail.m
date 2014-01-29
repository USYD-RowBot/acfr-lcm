function [x_jk,J] = tail2tail(x_ij,x_ik)
% DEPRECATED, use SSC_TAIL2TAIL instead.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-27-2006      rme         Renamed tail2tail.m to ssc_tail2tail.m

if (nargout == 1);
  x_jk = ssc_tail2tail(x_ij,x_ik);
else;
  [x_jk,J] = ssc_tail2tail(x_ij,x_ik);
end;
