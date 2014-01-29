function [x,y] = lblFixLS(XYZ,R,z);
%function [x,y] = lblFixLS(XYZ,z)
%
%  INPUTS:
%    XYZ     is a 3 x n array of East, North, Up beacon positions, the
%            first column corresponds to beacon A. [meters]
%    R       is a n-vector of slant ranges.
%    z       is the depth of transducer. [meters]
%
%  OUTPUTS:
%    x,y     are the East, North components of transducer position. [meters]
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-08-2006      rme         Created and written.

% Algorithm Reference
% P. Milne, Underwater Acoustic Positioning Systems, p. 71

% subtracting pairs of circle equations results in a linear system
% Ax=b
X = XYZ(1,:)';  Y = XYZ(2,:)';  Z = XYZ(3,:)';
R = [R(:)];                  % ensure slant ranges are in vector form
P = sqrt(R.^2 - (z - Z).^2); % project slant ranges onto XY plane
n = length(X);
jj = [2:n,1]';
ii = [1:n]';


% solve system
A = [(X(jj)-X(ii)), (Y(jj)-Y(ii))];
b = (P(jj).^2 - P(ii).^2)/2;
x = A \ b;

y = x(2);
x = x(1);
