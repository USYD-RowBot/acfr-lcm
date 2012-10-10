function [varargout] = fscatter3(X,Y,Z,C,cmap);
% [h,sel] = fscatter3(X,Y,Z,C,cmap);
% Plots point cloud data in cmap color classes and 3 Dimensions,
% much faster and very little memory usage compared to scatter3 !
% X,Y,Z,C are vectors of the same length
% with C being used as index into colormap (can be any values though)
% cmap is optional colourmap to be used
% h are handles to the line objects
% sel is optional output of data indices associated with handles

% Felix Morsdorf, Jan 2003, Remote Sensing Laboratory Zuerich
% Ryan Eustice, 20040319 added optional output of sel
% rme, 20060203 added caxis scaling and varargout
% rme, 20060724 made 'jet' cmap default and fixed bug in axis scaling
%               if minx and maxx are the same.
% rme, 20070510 fixed hold axis bug
  
if nargin == 4
  numclass = 256; % Number of color classes
  cmap = jet(256);
elseif nargin == 5
  numclass = max(size(cmap));
  if numclass == 1
    cmap = jet(256);
    numclass = 256;
  end  
end

% avoid too many calculations

mins = min(C);
maxs = max(C);
minz = min(Z);
maxz = max(Z);
minx = min(X);
maxx = max(X);
miny = min(Y);
maxy = max(Y);

% construct colormap :

col = cmap;

% determine index into colormap

%ii = round(interp1([floor(mins) ceil(maxs)],[1 numclass],C));
ii = round(interp1([mins maxs],[1 numclass],C)); % rme removed floor & ceil
nextPlotProp = get(gca,'NextPlot');
set(gca,'NextPlot','add'); % hold on
colormap(cmap);

% plot each color class in a loop
STORE_INDEX = false;
if nargout == 2
  STORE_INDEX = true;
end
k = 0;
for j = 1:numclass
  jj = find(ii == j);
  if ~isempty(jj)
    k = k + 1;
    h(k) = plot3(X(jj),Y(jj),Z(jj),'.','color',col(j,:), ...
		 'markersize',7);
    if STORE_INDEX
      sel{k} = jj;
    end
  end  
end
little = 3*eps;
axis([minx,maxx+little,miny,maxy+little,minz,maxz+little]);
axis image;
caxis([mins, maxs]);
set(gca,'NextPlot',nextPlotProp);

switch nargout;
case 0;
 % return nothing
case 1;
 varargout{1} = h;
case 2;
 varargout{1} = h;
 varargout{2} = sel;
otherwise;
 error('Incorrect number of output arguments');
end;
