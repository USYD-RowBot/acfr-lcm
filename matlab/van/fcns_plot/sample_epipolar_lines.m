function sample_epipolar_lines(I1,I2,u1,v1,u2,v2,F21)
%SAMPLE_EPIPOLAR_LINES draws epipolar geometry for image pair.
%   SAMPLE_EPIPOLAR_LINES(I1,I2,u1,v1,u2,v2,F21) regularly
%   samples the point correspondences (u1,v1) <--> (u2,v2) and draws the
%   corresponding epipolar lines.  F21 is the fundamental matrix defined
%   such that x2'*F21*x1 = 0.  FIGNUM optionally specifies the figure
%   number.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-03-2003      rme         Created from sample_ellipses.m and
%                                replaced the old version of
%                                sample_epipolar_lines.m

error(nargchk(7,8,nargin));

fignum = gcf;

nr = size(I1,1);
nc = size(I1,2);

udata = [1 nc]-1;
vdata = [1 nr]-1;

% sample I1 spatially
%-------------------------------------------
nx = 3; %5 
ny = 3; %3
nsamps = nx*ny;
xsamp1 = round(linspace(min(u1),max(u1),nx));
ysamp1 = round(linspace(min(v1),max(v1),ny));
[xsamp1,ysamp1] = meshgrid(xsamp1, ysamp1);
xsamp1 = xsamp1(:);
ysamp1 = ysamp1(:);

% find closest interest points to samples
%-------------------------------------------
tri = delaunay(u1,v1);
sel = dsearch(u1,v1,tri,xsamp1,ysamp1);
u1 = u1(sel);
v1 = v1(sel);
u2 = u2(sel);
v2 = v2(sel);


% plot pose instantiated epipolar lines
%--------------------------------------------
figure(fignum); clf;
set(fignum,'DoubleBuffer','on');
imagesc(udata,vdata,I1); colormap gray; axis image off;
title('I1');
hold on;
cvec = draw_epipolar_lines(F21',u2,v2,fignum,[nr nc]);
hold off;
figure(fignum+1); clf;
set(fignum+1,'DoubleBuffer','on');
imagesc(udata,vdata,I2); colormap gray; axis image off;
title('I2');
hold on;
cvec = draw_epipolar_lines(F21,u1,v1,fignum+1,[nr nc]);
hold off;

% plot interest points (u1,v1) on I1
%----------------------------------------
% note in this case it's faster to use two for loops vs. a single for
% loop.  this is because it slows matlab down to switch back and forth
% between the two figure windows.
figure(fignum);
hold on;
for ii = 1:nsamps
  plot(u1(ii),v1(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u1(ii),v1(ii),'o','color',cvec(ii,:),'linewidth',1.75);
end
hold off;

% plot interest points (u2,v2) on I2
%-----------------------------------------------------------------------
figure(fignum+1);
hold on;
for ii = 1:nsamps
  % plot the predicted point (u2p,v2p) on I2
  plot(u2(ii),v2(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u2(ii),v2(ii),'o','color',cvec(ii,:),'linewidth',1.75);  
end
hold off;

set(fignum,'DoubleBuffer','off');
set(fignum+1,'DoubleBuffer','off');
set(54,'Name','MLE Epipolar Geometry I1');
set(55,'Name','MLE Epipolar Geometry I2');
