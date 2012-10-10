function varargout = render_scene(X,Y,Z,map,u,v,Iorig)
%RENDER_SCENE displays a reconstructed surface.
%   RENDER_SCENE(X,Y,Z) renders the 3D set of points given by the [M x 1]
%   vectors X, Y, and Z.
%
%   RENDER_SCENE(X,Y,Z,MAP) allows the user to supply a color map.
% 
%   RENDER_SCENE(X,Y,Z,MAP,u,v,Iorig) displays a texture mapped reconstructed
%   surface.  X, Y, Z are [M x 1] vectors of 3D scene points in the
%   camera coordinate frame while u and v are [M x 1] vectors of their
%   imaged points.  Iorig is the original image of which the overlapping
%   part is rendered onto the 3D surface.  If MAP is left empty, i.e. [],
%   then the colormap defaults to gray.
%
%   [Xi,Yi,Zi] = RENDER_SCENE(...) returns the gridded scene surface.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-26-2003      rme         Created and written.

error(nargchk(3,7,nargin));
error(nargoutchk(0,3,nargout));  

if ~exist('map','var') && ~exist('Iorig','var')
  map = colormap('copper');
  u = [];
  v = [];
  Iorig = [];
elseif exist('map','var') && ~exist('Iorig','var')
  u = [];
  v = [];
  Iorig = [];  
elseif exist('Iorig','var') && isempty(map)
  map = colormap('gray');
end
  
% bounding box for scene
Xmin = min(X); Xmax = max(X);
Ymin = min(Y); Ymax = max(Y);

% sample scene points on a regular grid
Nsamps = 100;
[Xi,Yi] = meshgrid(linspace(Xmin,Xmax,Nsamps),linspace(Ymin,Ymax,Nsamps)');
Zi = griddata(X,Y,Z,Xi,Yi);

if isempty(Iorig)
  % generate lighted surface
  surfl(Xi,Yi,Zi);
  shading interp;  
  material dull;
  lighting gouraud;
  light('style','local');
else
  % bounding box for image
  % note top left image pixel is defined to be (0,0), therefore
  % add 1 to pixel coordinates to index into matlab correctly
  jmin = round(min(u))+1; jmax = round(max(u))+1;
  imin = round(min(v))+1; imax = round(max(v))+1;
  
  % extract region of image corresponding to scene
  Ioverlap = Iorig(imin:imax,jmin:jmax,:);

  % generate texture mapped surface
  warning off;
  surf(Xi,Yi,Zi,'Cdata',Ioverlap,'FaceColor','texturemap');
  shading flat;
  drawnow;
  warning on;
end
% orient display w.r.t. camera coordinates
set(gca,'Zdir','reverse','Xdir','reverse','Color',[0.8 0.8 0.8]);
axis equal;
grid off;
colormap(map);

% output
if nargout == 3
  varargout{1} = Xi;
  varargout{2} = Yi;
  varargout{3} = Zi;
end
