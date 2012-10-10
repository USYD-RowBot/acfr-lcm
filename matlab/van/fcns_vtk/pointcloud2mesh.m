function [X,Y,Z,binpop] = pointcloud2mesh(xyz,dx_grid,dy_grid,beam_width,beam_sigma,z_dir,xylim);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% [X,Y,Z,binpop] = pointcloud2mesh(xyz, dx_grid, dy_grid, beam_width, ...
%                                       beam_sigma, z_dir, xy_limits )
%
% X,Y,Z - matrices for surface plotting
% binpop - number of points in each grid bin
%
% optional:
% z_dir = 1 for z pos up and -1 for z pos down
% xy_limits = [min_x max_x min_y max_y]
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    cnr         Created and written.
%    01-13-2006      rme         Adapted and cleaned-up for VAN.
  

x = xyz(1,:)';
y = xyz(2,:)';
z = xyz(3,:)';
clear xyz;

% check for presence of optional argument
if ~exist('z_dir','var') || isempty(z_dir);
  z_dir = 1;
end;
if exist('xylim','var');
  ii = find( (xylim(1) <= x) & (x <= xylim(2)) & ...
	     (xylim(3) <= y) & (y <= xylim(4)));
  xyz = xyz(:,ii);
else;
  xylim = [min(x), max(x), min(y), max(y)];
end;


if z_dir < 0;
  z_flip = min(z);
  z = -(z-z_flip);    
end;
    
half_beam_width = beam_width/2;


xmin_grid = xylim(1) - roundunit(half_beam_width,dx_grid);
xmax_grid = xylim(2) + roundunit(half_beam_width,dx_grid);
ymin_grid = xylim(3) - roundunit(half_beam_width,dy_grid);
ymax_grid = xylim(4) + roundunit(half_beam_width,dy_grid);

nx = ceil(((xmax_grid - xmin_grid)/dx_grid)) + 1;
ny = ceil(((ymax_grid - ymin_grid)/dy_grid)) + 1;

half_beam_width_bins_x = ceil(half_beam_width / dx_grid);
half_beam_width_bins_y = ceil(half_beam_width / dy_grid);

zmax = max(z);
zmin = min(z);

Z = zeros(ny,nx);
binpop = Z;

xi = linspace(xmin_grid,xmax_grid,nx);
yi = linspace(ymin_grid,ymax_grid,ny);

mapXdim = xmax_grid - xmin_grid;
mapYdim = ymax_grid - ymin_grid;
fprintf('Bathymetric map: %+.2f [m] E-W  by %+.2f [m] N-S with total area %+.2f [m^2]\n', ...
	mapXdim,mapYdim,mapXdim*mapYdim);
fprintf('Data range is X=[%+.2f,%+.2f]   Y=[%+.2f,%+.2f]   Z=[%+.2f,%+.2f]\n', ...
	min(x),max(x),min(y),max(y),zmin,zmax);   
fprintf('Grid range is X=[%+.2f,%+.2f]   Y=[%+.2f,%+.2f]   Z=[%+.2f,%+.2f]\n', ...
	xmin_grid,xmax_grid,ymin_grid,ymax_grid,zmin,zmax);   
fprintf('Gridding bathymetry into a %d x %d grid of cells measuring %.2f x %.2f \n', ...
	nx,ny,dx_grid,dy_grid);
fprintf('beam_width      is %.2f meters, %.2f X bins, %.2f Y bins\n', ...
	beam_width, 2*half_beam_width_bins_x, 2*half_beam_width_bins_y);
fprintf('half_beam_width is %.2f meters, %.2f X bins, %.2f Y bins\n', ...
	half_beam_width, half_beam_width_bins_x, half_beam_width_bins_y);
fprintf('beam_sigma      is %.2f meters.\n',beam_sigma);


[X,Y] = meshgrid(xi, yi);

rowindex_center = round( ((y - ymin_grid)*(1/dy_grid))) + 1;
colindex_center = round( ((x - xmin_grid)*(1/dx_grid))) + 1;

fprintf(1,'Processing %d data points\n',length(x)); 
xpercent = floor(length(x)/20);

% construct beam pattern mask
xm         = linspace(-half_beam_width,+half_beam_width,(2*half_beam_width_bins_x)+1);
ym         = linspace(-half_beam_width,+half_beam_width,(2*half_beam_width_bins_y)+1);
[Xm, Ym]   = meshgrid(xm, ym);
Zm         = 1/(2*pi*beam_sigma^2)*exp(-0.5*(Xm.^2 + Ym.^2)/beam_sigma^2);
Zm         = Zm / sum(Zm(:)); % normalize finite kernel window to sum to unity

% convolve the kernel with point cloud
for kk = 1:length(x);
  
  ii = (rowindex_center(kk)-half_beam_width_bins_y):(rowindex_center(kk)+half_beam_width_bins_y);
  jj = (colindex_center(kk)-half_beam_width_bins_x):(colindex_center(kk)+half_beam_width_bins_x);
  
  Z(ii,jj)  = Z(ii,jj) + (Zm * z(kk));
  binpop(ii,jj) = binpop(ii,jj) + Zm;
  
  if mod(kk,xpercent) == 0;
    fprintf('\rProcessed  %d (%.0f%%) points ...   ',kk,100*kk/length(x));
  end;
end;
fprintf('DONE.\n');

zerobins = (binpop == 0);
binpop(zerobins) = NaN;
Z(zerobins) = NaN;


%do the averaging 
Z = Z ./ binpop;

% Clip the margins of the output matrix back to the size of the actual data points
ii = half_beam_width_bins_y+1:ny-half_beam_width_bins_y;
jj = half_beam_width_bins_x+1:nx-half_beam_width_bins_x;

X = X(ii,jj);
Y = Y(ii,jj);
Z = Z(ii,jj);
binpop = binpop(ii,jj);

if z_dir < 0;
  Z = -Z + z_flip;  
end; 
