function [x_ji,Jminus] = inverse(x_ij);
% DEPRECATED, use SSC_INVERSE instead.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-27-2006      rme         Renamed inverse.m to ssc_inverse.m

if (nargout == 1);
  x_ji = ssc_inverse(x_ij);
else;
  [x_ji,Jminus] = ssc_inverse(x_ij);
end;
