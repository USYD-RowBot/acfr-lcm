function plot_owtt(nav_t,top_t,abstime,rovortoa);
%function plot_owtt(nav_t,top_t,abstime,rovortoa);
%
%   Plots OWTT travel times from Trackpoint and uModems.
%
%   abstime is an optional flag to plot versus absolute unixtime,
%   o/w defaults to mission time.
%
%   rovvstoa is an optional string {'rovtime','toa'} that selects
%   which independent variable for CATOA display.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-24-2006      rme         Created and written.

if ~exist('abstime','var') || isempty(abstime); abstime = false; end;
if ~exist('rovortoa','var') || isempty(rovortoa); rovortoa = 'rovtime'; end;

toptimeLXT = top_t.LXT.rovtime + top_t.STARTTIME;
switch lower(rovortoa(1:3));
case 'rov';
 toptimeTOA = top_t.CATOA.rovtime + top_t.STARTTIME;
 navtimeTOA = nav_t.CATOA.rovtime + nav_t.STARTTIME;
 xsrc = '(rovtime)';
case 'toa';
 toptimeTOA = top_t.CATOA.toa;
 navtimeTOA = nav_t.CATOA.toa;
 xsrc = '(toa)';
otherwise;
 error('rovortoa is nkown, must be one of ''rovtime'' or ''toa''');
end;

topowttLXT = top_t.LXT.rng/1500;
topowttTOA = top_t.CATOA.owtt;
navowttTOA = nav_t.CATOA.owtt;

if abstime;
  xstring = ['Unix Time [s] ',xsrc];
else;
  to = min([nav_t.STARTTIME,top_t.STARTTIME]);
  toptimeLXT = toptimeLXT - to;
  toptimeTOA = toptimeTOA - to;
  navtimeTOA = navtimeTOA - to;;
  xstring = ['Mission Time [s] ',xsrc];
end;

fig = gcf;
figure(fig);
subplot(2,1,1);
set(gca,'fontsize',20);
plot(toptimeLXT,topowttLXT,'k.',...
	 toptimeTOA,topowttTOA,'g.',...
	 navtimeTOA,navowttTOA,'r.');
xlabel(xstring);
ylabel('OWTT [s]');
legend('Trackpoint','Topside Modem','Vehicle Modem');
title('Raw OWTTs');
grid on;

% reject outliers via median filtering
N = 6;
TOL = 0.01;
topowttLXT(abs(topowttLXT-medfilt1(topowttLXT,N)) > TOL) = NaN;
topowttTOA(abs(topowttTOA-medfilt1(topowttTOA,N)) > TOL) = NaN;
navowttTOA(abs(navowttTOA-medfilt1(navowttTOA,N)) > TOL) = NaN;
  
subplot(2,1,2);
set(gca,'fontsize',20);
plot(toptimeLXT,topowttLXT,'k.',...
	 toptimeTOA,topowttTOA,'g.',...
	 navtimeTOA,navowttTOA,'r.');
xlabel(xstring);
ylabel('OWTT [s]');
legend('Trackpoint','Topside Modem','Vehicle Modem');
title('Median Filtered OWTTs');
grid on;


% interpolate trackpoint to modem times
[tmp,ii] = unique(toptimeLXT);
topowttLXTi = interp1(toptimeLXT(ii),topowttLXT(ii),toptimeTOA,'linear');
navowttLXTi = interp1(toptimeLXT(ii),topowttLXT(ii),navtimeTOA,'linear');

% plot histograms of error
figure(fig+1);
set(gca,'fontsize',20);
toperror = (topowttTOA-topowttLXTi)*1e3;
hist(toperror,50);
xlabel('Topside Modem Error w.r.t. Trackpoint [ms]');
title(sprintf('Sample Mean: %.3f   Std: %.3f',nanmean(toperror),nanstd(toperror)));

figure(fig+2);
set(gca,'fontsize',20);
naverror = (navowttTOA-navowttLXTi)*1e3;
hist(naverror,50);
xlabel('Vehicle Modem Error w.r.t. Trackpoint [ms]');
title(sprintf('Sample Mean: %.3f   Std: %.3f',nanmean(naverror),nanstd(naverror)));