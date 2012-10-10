function [x_ik,Jplus] = head2tail(x_ij,x_jk)
% DEPRECATED, use SSC_HEAD2TAIL instead.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-27-2006      rme         Renamed head2tail.m to ssc_head2tail.m

if (nargout == 1);
  x_ik = ssc_head2tail(x_ij,x_jk);
else;
  [x_ik,Jplus] = ssc_head2tail(x_ij,x_jk);
end;
