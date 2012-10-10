function [fovx,fovy] = fov(K,imgsize);
%FOV Image field-of-view.
%   [FOVX,FOVY] = FOV(K,IMGSIZE) computes the image field-of-view
%   in degrees for the x and y directions, respectively, given the
%   [3 x 3] camera calibration matrix K and the 2-vector IMGSIZE,
%   which is the image resolution in pixels (i.e. IMGSIZE = [NR,NC]).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-22-2006      rme         Created and written.

%  image size
%  ----------U3---------
%  |                   |
%  |                   |
%  U1                  U2
%  |                   |
%  |                   |
%  ----------U4---------  
%

% x-dir image extrema, along COP centerline
U1 = [0; K(2,3); 1];
U2 = [imgsize(2)-1; K(2,3); 1];

% y-dir image extrema, along COP centerline
U3 = [K(1,3); 0; 1];
U4 = [K(1,3); imgsize(1)-1; 1];

% normalize
Kinv = K^-1;
[x1,y1] = dehomogenize(Kinv * U1);
[x2,y2] = dehomogenize(Kinv * U2);
[x3,y3] = dehomogenize(Kinv * U3);
[x4,y4] = dehomogenize(Kinv * U4);

% in normalized coordinates, compute FOV
fovx = (abs(atan(x1/1)) + abs(atan(x2/1))) * RTOD;
fovy = (abs(atan(y3/1)) + abs(atan(y4/1))) * RTOD;

