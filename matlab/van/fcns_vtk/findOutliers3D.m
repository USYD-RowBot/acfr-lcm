function ii = findOutliers3D(xyz,X,Y,Z,tolband);
%function ii = findOutliers3D(xyz,X,Y,Z,tolband);
%
%  xyz     is a 3 x M array of irregulary spaced points
%  X,Y,Z   are regularly spaced points are from meshgrid
%  tolband is the acceptable tolerance band where the error is defined as:
%          e = Z(X,Y) - z(x,y);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-13-2006      rme         Created and written.
  
% ensure column-vectors
x = xyz(1,:)';
y = xyz(2,:)';
z = xyz(3,:)';
clear xyz;

% compute selection index of points within tol band
Zi = interp2(X,Y,Z,x,y,'linear');
ii = abs(Zi-z)<tolband;
