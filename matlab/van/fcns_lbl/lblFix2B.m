function [x,y,soln] = lblFix2B(XYZ,R,z,cwflag);
%function [x,y,soln] = lblFix2B(XYZ,z,cwflag)
%
%  INPUTS:
%    XYZ     is a 3 x 2 array of East, North, Up beacon positions, the
%            first column corresponds to beacon A. [meters]
%    R       is a 2-vector of slant ranges.
%    z       is the depth of transducer. [meters]
%    cwflag  is the baseline flag, CW + 1, CCW -1.  
%
%  OUTPUTS:
%    x,y     are the East, North components of transducer position. [meters]
%    soln    is a scalar indicating which solution is being returned,
%            +1 positive, -1 negative
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-08-2006      rme         Created and written.

% Algorithm Reference
% P. Milne, Underwater Acoustic Positioning Systems, p. 67
R1 = R(1);      R2 = R(2);
X1 = XYZ(1,1);  X2 = XYZ(1,2);
Y1 = XYZ(2,1);  Y2 = XYZ(2,2);
Z1 = XYZ(3,1);  Z2 = XYZ(3,2);

% slant ranges projected onto X-Y plane
P1 = sqrt(R1^2 - (Z1 - z)^2);
P2 = sqrt(R2^2 - (Z2 - z)^2);

% compute intermediate position fix variables
A = (X2-X1);
B = (Y2-Y1);
C = (X2^2-X1^2) + (Y2^2-Y1^2) - (P2^2-P1^2);
a = 4*B^2 + 4*A^2;
b = 8*A*B*X1 - 8*A^2*Y1 - 4*B*C;
c = 4*(X1^2+Y1^2-P1^2)*A^2 - 4*X1*A*C + C^2;

% solution 1
q  = sqrt(b^2 - 4*a*c);
if ~isreal(q);
  warning('Invalid LBL geometry');
  x = NaN; y = NaN; soln = NaN;
  return;
end;
y1 = (-b + q) / (2*a);
x1 = (C - 2*y1*B) / (2*A);

% solution 2
y2 = (-b - q) / (2*a);
x2 = (C - 2*y2*B) / (2*A);

% return the solution that lies on the correct side of the baseline
% using the cross-product between the baseline and solution to determine
% which side
baseline = [A,B,0]';        % baseline vector
soln1 = [x1-X1,y1-Y1,0]';   % solution 1 vector
cp = cross(soln1,baseline); % vector cross-product
if sign(cp(3)) == sign(cwflag);
  x = x1;  y = y1;  soln = 1;
else;
  x = x2;  y = y2;  soln = 2;
end;

