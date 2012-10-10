function [xn,yn] = oulu_inverse_distortion(a,xd,yd)
%OULU_INVERSE_DISTORTION  Applies the Heikkila inverse distortion model.
%  [XN,YN] = OULU_INVERSE_DISTORTION(A,XD,YD) uses the [8x1]
%  parameter vector A and the inverse distortion model proposed by
%  Heikkila to compensate for radial distortion.  XD and YD are
%  [Nx1] vectors of distorted normalized image coordinates.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-14-2002      rme         Created and written.

a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
a6 = a(6);
a7 = a(7);
a8 = a(8);

r2 = xd.^2 + yd.^2;
r4 = r2.*r2;
r6 = r4.*r2;

G = 1 + r2.*(a5*r2 + a6*xd + a7*yd + a8);

xn = (xd + xd.*(a1*r2+a2*r4) + 2*a3*xd.*yd + a4*(r2+2*xd.^2))./G;
yn = (yd + yd.*(a1*r2+a2*r4) + a3*(r2+2*yd.^2) + 2*a4*xd.*yd)./G;
