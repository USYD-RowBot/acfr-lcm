function [X,Y,tag] = sim_scene_xy(fpi,xlim,ylim,alt,K,res,samptype);
%SIM_SCENE_XY generates XY coordiantes of 3D feature points consistent with
%             camera geometry
%   [X,Y,TAG] = SIM_SCENE_XY(FPI,XLIM,YLIM,ALT,K,RES) generates [Nf x 1]
%   3D scene coordinates X,Y and unique label TAG to yield uniform
%   coverage over the user specified area.  Input arguments are:
%    FPI  : features per image
%           (based upon calculation of image footprint over a flat bottom)
%    XLIM : [min,max] limits of scene in E-W dir [m]  
%    YLIM : [min,max] limits of scene in N-S dir [m]
%    ALT  : intended vehicle altitude from scene [m]
%    K    : [3 x 3] camera calibration matrix
%    RES  : [resx,resy] pixel resolution of camera
%
%-----------------------------------------------------------------
%    History:
%    Date          Who        What
%    -----------   -------    -----------------------------
%    20031128      OP         Simplified from generate_scene
%    20031219      rme        Ryanized

if ~exist('samptype','var')
  samptype = 'uniform';
end

% density of features per square meter
fpm2 = fdensity(fpi,alt,K,res);

xdim = xlim(2)-xlim(1);
ydim = ylim(2)-ylim(1);
Nf = round((xdim*ydim)*fpm2);

switch samptype
 case 'uniform'
  X = xdim*rand(Nf,1) + xlim(1);
  Y = ydim*rand(Nf,1) + ylim(1);
 case 'grid'
  X = linspace(xlim(1),xlim(2),sqrt(Nf));
  Y = linspace(ylim(1),ylim(2),sqrt(Nf));
  [X,Y] = meshgrid(X,Y);
  keyboard;
  X = X(:);
  Y = Y(:);
end

% unique label associated to each 3D feature
tag = [1:Nf]';



%-------------------------------------------------------------
function fpm2 = fdensity(fpi,alt,K,res)
% calculates the density of 3D features per square meter given the
% desired number of features per image and the camera geometry.
% this is done through the footprint area assuming a flat bottom.
resx = res(1);
resy = res(2);

% rays corresponding to [0,0] (top left corner) and 
% [resx-1,resy-1] (bottom right corner)
invK = inv(K);
xtl = invK*[0;0;1];
xbr = invK*[resx-1;resy-1;1];

% footprint dimensions
xdim = alt*(xbr(1) - xtl(1));
ydim = alt*(xbr(2) - xtl(2));

% area of image footprint
area = xdim*ydim;

% density of features per square meter
fpm2 = fpi/area;
