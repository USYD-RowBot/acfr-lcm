function [X,Y,Z,sel] = rand3Dsurface(Nf,xdim,ydim,zstd,smoothness)
%function [X,Y,Z,sel] = generate3Dsurface(Nf,xdim,ydim,zstd,smoothness)  
% X,Y,Z are matrices suitable for surf
% sel is a *linear* index into these matrices to produce a random
% sampling of points from the surface
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created and written.

if ~exist('smoothness','var')
  smoothness = 0.5; % default
end
  
if smoothness < 0 || smoothness > 1
  error('smoothness must be in range [0,1]');
end

r = xdim/ydim; % surface aspect ratio 
% i calculated the surface resolution based upon the following add-hoc
% equations: Nx*Ny = 10*Nf with constraint Nx = r*Ny
Ny = sqrt(10/r*Nf);
Nx = r*Ny;
Nx = ceil(Nx); Ny = ceil(Ny);
x = linspace(-xdim/2,xdim/2,Nx);
y = linspace(-ydim/2,ydim/2,Ny);
[X,Y] = meshgrid(x,y);

% smoothing filter
hsize = ceil(1.25*smoothness*max(length(x),length(y)));
hsize = max(hsize,1); % minimum size is an impulse;
sigma = hsize/6;
g = fspecial('gaussian',hsize,sigma);

% random matrix
Z = (rand(size(X)+hsize-1)-0.5)*sqrt(12); % normalized to unit-variance

% "smooth" surface
Z = conv2(Z,g,'valid');

Z = zstd*Z/std(Z(:));

% randomly sample surface at Nf points
sel = unique(ceil(prod(size(Z))*rand(Nf,1)));
while length(sel) < Nf
  tmp = unique(ceil(prod(size(Z))*rand(Nf-length(sel),1)));
  for ii=1:length(tmp)
    if ~any(sel == tmp(ii))
      sel = [sel;tmp(ii)];
    end
  end
end
