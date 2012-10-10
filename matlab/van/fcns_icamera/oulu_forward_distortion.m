function [xd,yd] = oulu_forward_distortion(kr,kt,xn,yn)
%OULU_FORWARD_DISTORTION  Applies the Heikkila forward distortion model.
%  [XD,YD] = OULU_FORWARD_DISTORTION(KR,KT,XN,YN) applies the
%  distortion model to undistorted normalized image coordinates
%  represented by the [Nx1] vectors XN,YN.  KR is the [3x1] vector of
%  radial distortion coefficients and KT is the [2x1] vector of
%  tangential distortion.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-14-2002      rme         Created and written.

k1 = kr(1);
k2 = kr(2);
k3 = kr(3);
p1 = kt(1);
p2 = kt(2);

rn2 = xn.^2 + yn.^2;
rn4 = rn2.*rn2;
rn6 = rn4.*rn2;

% apply radial distortion model to undistorted normalized points
dr = k1*rn2 + k2*rn4 + k3*rn6;
xd = xn.*(1+dr);
yd = yn.*(1+dr);

% apply tangential distortion model if necessary
if p1~=0 || p2~=0
  xd = xd + (2*p1*xn.*yn + p2*(rn2+2*xn.^2));
  yd = yd + (p1*(rn2+2*yn.^2) + 2*p2*xn.*yn);
end
