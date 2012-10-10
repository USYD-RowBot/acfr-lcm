function sample_ellipses(I1,I2,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,F21,chiSquare2dof)
%SAMPLE_ELLIPSES draws point transfer uncertainty ellipses.
%   SAMPLE_ELLIPSES(I1,I2,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,F21,CHISQUARE)
%   samples interest points (u1,v1) in I1 and plots the uncertainty ellipses
%   centered around (u2p,v2p) in I2.  Interest points (u2,v2) which fall
%   within these ellipses are also shown.  Cov_u2pv2p is a [2 x 2 x N] array with
%   each element being the [2 x 2] covariance matrix of (u2p,v2p).  The
%   fundamental matrix F21 is defined such that x2'*F21*x1 = 0.  CHISQUARE2DOF is
%   the chi-square random variable with 2 degrees of freedom which specified the
%   bounding search ellipse.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            rme         Created.
%    09-27-2003      rme         Added help text.
%    11-26-2004      rme         Updated call to draw_ellipses.m
%    12-18-2004      rme         Removed fignum argument.
%    12-22-2004      rme         Changed Cov_u2pv2p from a cell to a [2 x 2 x N] array  
%    12-25-2004      rme         Modified to pass chiSquare2dof as an input argument instead
%                                of confidence level alpha.  This makes it easier to
%                                scale the bounding ellipse size across multiple functions.
%    01-12-2006      rme         Added alternative xsamp1,ysamp1

fignum = gcf;
nr = size(I1,1);
nc = size(I1,2);

udata = [1 nc]-1;
vdata = [1 nr]-1;

% sample I1 spatially
%-------------------------------------------
if false;
  nx = 3;
  ny = 3;
  xsamp1 = round(linspace(0.1*nc,0.9*nc,nx));
  ysamp1 = round(linspace(0.1*nr,0.9*nr,ny));
  [xsamp1,ysamp1] = meshgrid(xsamp1, ysamp1);
else;
  xsamp1 = nc*[0.10,0.50,0.90,0.30,0.70,0.10,0.50,0.90];
  ysamp1 = nr*[0.10,0.20,0.10,0.50,0.50,0.90,0.90,0.90];
end;
xsamp1 = xsamp1(:);
ysamp1 = ysamp1(:);
nsamps = length(xsamp1);

% find closest interest points to samples
%-------------------------------------------
[tmp,sel] = unique([u1(:), v1(:)], 'rows','first');
u1 = u1(sel);
v1 = v1(sel);
u2p = u2p(sel);
v2p = v2p(sel);
tri = delaunay(u1,v1);
sel = dsearch(u1,v1,tri,xsamp1,ysamp1);
u1  = u1(sel);
v1  = v1(sel);
u2p = u2p(sel);
v2p = v2p(sel);
Cov_u2pv2p = Cov_u2pv2p(:,:,sel);


% plot pose instantiated epipolar lines
%--------------------------------------------
figure(fignum); clf;
set(fignum,'DoubleBuffer','on');
imagesc(udata,vdata,I1); colormap gray; axis image off; title('I1');
hold on;
cvec = draw_epipolar_lines(F21',u2p,v2p,fignum,[nr nc]);
hold off;

figure(fignum+1);
set(fignum+1,'DoubleBuffer','on');
imagesc(udata,vdata,I2); colormap gray; axis image off; title('I2');
hold on;
cvec = draw_epipolar_lines(F21,u1,v1,fignum+1,[nr nc]);
hold off;

% plot interest points (u1,v1) on I1
%----------------------------------------
% note in this case it's faster to use two "for loops" vs. a single "for
% loop".  this is because it slows matlab down to switch back and forth
% between the two figure windows.
figure(fignum);
hold on;
for ii = 1:nsamps
  plot(u1(ii),v1(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u1(ii),v1(ii),'o','color',cvec(ii,:),'linewidth',1.75);
end
hold off;

% plot uncertainty ellipses based upon results of two-vew point transfer
%-----------------------------------------------------------------------
figure(fignum+1);
hold on;
% bounding ellipse is specified by chi-squared random variable
% with 2 degrees of freedom
% chiSquare2dof is chi squared random variable with 2 DOF
for ii = 1:nsamps
  % plot the predicted point (u2p,v2p) on I2
  plot(u2p(ii),v2p(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u2p(ii),v2p(ii),'o','color',cvec(ii,:),'linewidth',1.75);  
  % plot the search ellipse centered on (u2p,v2p) on I2
  Sigma = squeeze(Cov_u2pv2p(:,:,ii));
  draw_ellipse([u2p(ii); v2p(ii)],Sigma,chiSquare2dof,'-.','color',cvec(ii,:));
  % plot the points from (u2,v2) which lie inside the ellipse
  if true
    % bounding ellipse is given by:
    % (u-u2p)'*Cov^-1*(u-u2p) = chiSquare2dof
    u_error = u2-u2p(ii);
    v_error = v2-v2p(ii);
    Lambda  = Sigma^-1;
    epsilon =     Lambda(1,1)*u_error.*u_error ...
	      + 2*Lambda(1,2)*u_error.*v_error ...
	      +   Lambda(2,2)*v_error.*v_error;
  
    % find points (u2,v2) which lie inside the ellipse
    sel = find(epsilon <= chiSquare2dof);
    plot(u2(sel),v2(sel),'.','color',cvec(ii,:));
  end;
end;
hold off;

set(fignum,'DoubleBuffer','off');
set(fignum+1,'DoubleBuffer','off');
