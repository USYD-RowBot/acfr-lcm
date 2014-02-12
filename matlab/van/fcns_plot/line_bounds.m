function [xx,yy] = line_bounds(xminus,xplus,yminus,yplus,epl);
%LINE_BOUNDS Purpose is to keep line endpoints within specified boundary.
%   [XX,YY] = LINE_BOUNDS(XMINUS,XPLUS,YMINUS,YPLUS,L) returns
%   [2x1] vectors XX and YY such that line(XX,YY) will plot a
%   line.  L is a 3-vector of homogenous line parameters.  XMINUS,
%   XPLUS, YMINUS, and YPLUS specify the bounding box.
%
%   Purpose is to keep line bounds within image boundary.  Matlab
%   appears to have some type of bug under linux where if you specify
%   the line endpoints outside of the current axis boundaries, then
%   the specified line color may not show up.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-06-2002      rme         Created and written.


% line equation:
% epl(1)*x + epl(2)*y + epl(3) = 0
if abs(epl(2)) > 1e-6; 
  xx(1) = xminus; yy(1) = -(epl(1)*xminus + epl(3))/epl(2);
  xx(2) = xplus;  yy(2) = -(epl(1)*xplus  + epl(3))/epl(2);
else % x ~= constant
  xx(1) = xminus; yy(1) = -inf;
  xx(2) = xplus;  yy(2) = +inf;
end
if abs(epl(1)) > 1e-6;
  yy(3) = yminus; xx(3) = -(epl(2)*yminus + epl(3))/epl(1);
  yy(4) = yplus;  xx(4) = -(epl(2)*yplus  + epl(3))/epl(1);
else % y ~= constant
  yy(3) = yminus; xx(3) = -inf;
  yy(4) = yplus;  xx(4) = +inf;
end

[xx,ii] = sort(xx);
yy = yy(ii);

% points on line within image boundaries
xx = [xx(2), xx(3)]';
yy = [yy(2), yy(3)]';
