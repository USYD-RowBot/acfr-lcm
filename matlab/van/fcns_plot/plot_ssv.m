function plot_ssv(nav_t,ssv_t,firsttime,showsigma);
%function plot_ssv(nav_t,ssv_t,firsttime,showsigma);
%
%  plot_ssv(nav_t,ssv_t);           % normal syntax
%  plot_ssv(nav_t,ssv_t,true,true); % force firsttime syntax
%
%History
%Date          Who        Comment
%----------    ---        -----------------------------------
%2003/08/14    rme        Create
%2004/03/09    rme        Updated to include PHINS and plot cooked values.
%2004-04-10    rme        Modified to archive Xv instead of Xp
%2004-07-01    rme        Plot xy trajectory estimate with line instead
%                         of points
%2004-11-03    rme        Restructured to work with reorganized ssv_t structure.
%2006-01-09    rme        Added firsttime calculation.

persistent tmin tmax;

if ~exist('firsttime','var') || isempty(firsttime);
  firsttime = false;
end;
if ~exist('showsigma','var') || isempty(showsigma);
  showsigma = true;
end;

if isempty(tmin) || (ssv_t.t(1) <= tmin+1)
  firsttime = true;
else;
  firsttime = false;
end;

twin   = [-30 0];  % size of sliding time window [seconds]
ecolor = 'm';      % error bar color
pcolor = 'c';      % estimate color

% shorthand index
Xp_i = ssv_t.Index.Xp_i;  % pose elements index
Xe_i = ssv_t.Index.Xe_i;  % extraneous elements index
xi = Xp_i(1);
yi = Xp_i(2);
zi = Xp_i(3);
ri = Xp_i(4);
pi = Xp_i(5);
hi = Xp_i(6);

% define sliding time window of interest
cc = ssv_t.cc-1;
if isempty(tmin) || firsttime;
  tmin = ssv_t.t(1);
else;
  tmin = tmax;
end;
tmax = ssv_t.t(cc);
ti = find( (tmin < ssv_t.t) & (ssv_t.t <= tmax) );
if isempty(ti);
  return;
end;

%===========================================================
% XY Plot
%===========================================================
% select a nav sensor for comparison
if isfield(nav_t,'LBL');
  navXY = nav_t.LBL;
  legendString = {'lbl','est'};
elseif isfield(nav_t,'RDI');
  navXY = nav_t.RDI;
  legendString = {'rdi','est'};
else;
  error('Missing XY nav sensor');
end;

% find index corresponding to current time window of interest
ii = find( (tmin < navXY.rovtime) & (navXY.rovtime < tmax) );

% plot XY filtered results 
% account for world to local-level coordinate xform by swapping x/y axes
fignum = gcf;
figure(fignum);
if firsttime; clf; end;
hold on;
handles.xy = plot(navXY.nx(ii), navXY.ny(ii), 'b.', ...
		  ssv_t.mu_x(yi,ti), ssv_t.mu_x(xi,ti), strcat(pcolor,'--'));  
axis equal;
grid on;
if firsttime; legend(legendString{:}); end;
hold off;
% draw uncertainty ellipses at sampled time instances
if showsigma;
  hold on;
  nSigma = 3;
  alpha = 1 - 2*normcdf(-nSigma);
  k2 = chi2inv(alpha,2);
  for i=1:length(ti);
    % extract the pose mean and covariance
    mu    = ssv_t.mu_x([yi,xi],ti(i));
    Sigma = squeeze( ssv_t.Sigma_xx([yi,xi],[yi,xi],ti(i)) );
    
    % plot the covariance ellipse
    draw_ellipse(mu,Sigma,k2,ecolor);
    plot(mu(1),mu(2),strcat('+',ecolor));
  end;
  hold off;
else;
  nSigma = 0;
end;
xlabel('East [m]');
ylabel('North [m]');
title(sprintf('Filtered Vehicle Trajectory %d-sigma',nSigma));
set(fignum,'Name','Filtered Vehicle Trajectory');

%===========================================================
% Depth plot
%===========================================================
% find index corresponding to current time window of interest
ii = find( (tmin < nav_t.PARO.rovtime) & (nav_t.PARO.rovtime < tmax) );

% plot depth Z filtered results
figure(fignum+1);
if firsttime; clf; end;
hold on;
Handles.depth = plot(nav_t.PARO.rovtime(ii), -nav_t.PARO.depth_cooked(ii), 'b.', ...
		     ssv_t.t(ti), -ssv_t.mu_x(zi,ti), strcat(pcolor,'--'));
if showsigma;
  errorbar(ssv_t.t(ti), -ssv_t.mu_x(zi,ti), nSigma*sqrt(ssv_t.Sigma_xx(zi,zi,ti)), strcat(ecolor,'.'));
end;
grid on;
set(gca,'Xlim',[tmin tmax]+twin);
xlabel('mission time [s]');
ylabel('depth [m]');
title(sprintf('Filtered Vehicle Depth %d-sigma',nSigma));
if firsttime; legend('nav','est'); end;
hold off;
set(fignum+1,'Name','Filtered Vehicle Depth');

%===========================================================
% Heading plot
%===========================================================
% select a nav sensor for comparison
if isfield(nav_t,'PHINS');
  navHdg = nav_t.PHINS;
  legendString = {'phins','est'};
elseif isfield(nav_t,'RDI');
  navHdg = nav_t.RDI;
  legendString = {'rdi','est'};
else;
  error('Missing heading nav sensor');
end;

% find index corresponding to current time window of interest
ii = find( (tmin < navHdg.rovtime) & (navHdg.rovtime < tmax) );

% plot filtered heading results
figure(fignum+2);
if firsttime; clf; end;
hold on;
plot(navHdg.rovtime(ii), navHdg.heading_cooked(ii)*RTOD, 'b.', ...
     ssv_t.t(ti), ssv_t.mu_x(hi,ti)*RTOD, strcat(pcolor,'--'));
if showsigma;
  errorbar(ssv_t.t(ti),ssv_t.mu_x(hi,ti)*RTOD,nSigma*sqrt(ssv_t.Sigma_xx(hi,hi,ti))*RTOD,strcat(ecolor,'.'));
end;
grid on;
set(gca,'Xlim',[tmin tmax]+twin);
xlabel('mission time [s]');
ylabel('heading [degrees]');
title(sprintf('Filtered Vehicle Heading %d-sigma',nSigma));
if firsttime; legend(legendString{:}); end;
hold off;
set(fignum+2,'Name','filtered Vehicle Heading');

% set persistent initialization flag for next pass
initialized = true;
