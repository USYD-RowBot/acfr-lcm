function Icartesian = zernikePolar2Cartesian(Ipolar,patchSize,Zbasis);
%function Icartesian = zernikePolar2Cartesian(Ipolar,patchSize,Zbasis);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.

% uniform cartesian sample grid
[xgrid,ygrid] = meshgrid( linspace(-1,1,patchSize(2)), linspace(1,-1,patchSize(1)) );

% sample points in input image space
dr = diff(Zbasis.rsamp(1,1:2));
dt = diff(Zbasis.tsamp(1:2,1));
rgrid = sqrt(xgrid.*xgrid + ygrid.*ygrid) / dr;% [dr,1]
tgrid = mod(atan2(ygrid,xgrid),2*pi) / dt + 1; % [0,2pi-dt]

% create a resampler structure
R = makeresampler('linear','fill');

% sample the raw image directly by specifying the raw image space sample points
% with a tmap_b data structure used within the tformarray command
tmap_b = cat(3,rgrid,tgrid);
Icartesian = tformarray(Ipolar,[],R,[2 1],[1 2],[],tmap_b,0);
