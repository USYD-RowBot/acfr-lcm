function F = F_from_KRt(R,t,K)
%F_from_KRt computes the fundamental matrix based upon relative pose.
%   F = F_from_KRt(R,t,K) R is the [3x3] rotation matrix, t is the [3x1]
%   translation and K is the [3x3] camera calibration matrix.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-30-2003      rme         Created and written.
%    09-15-2003      rme         Renamed to F_from_Rt.m to F_from_KRt.m

Kinv = inv(K);
  
F = transpose(Kinv)*skewsym(t)*R*Kinv;
