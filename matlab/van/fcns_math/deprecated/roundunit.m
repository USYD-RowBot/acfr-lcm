function y = roundunit(x,base)
% DEPRECATED, use ROUNDB instead.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-28-2006      rme         Renamed roundunit.m to roundb.m

warning('deprecated, use roundb() instead.');

y = roundb(x,base);
