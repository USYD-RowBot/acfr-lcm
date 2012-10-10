function xylatlonplt()
cla;
legend_str='                                                                 ';
hold on


% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

if exist('iver_t','var')
    LATLON = getclim;
    if ~exist('LATLON','var') || isempty(LATLON)
      LATLON = [0 0];
      setclim(LATLON);
    end
    plot(iver_t.gps.Longitude, iver_t.gps.Latitude,':','LineWidth',2);
    legend_str=strvcat(legend_str,'GPS (iver)');
end



% lcmlog
try
    nav_t = evalin('base','nav_t');
end

if exist('nav_t','var')
    if (isfield (nav_t, 'GPSD'))
        plot(nav_t.GPSD.longitude.*RTOD, nav_t.GPSD.latitude.*RTOD,'r.','LineWidth',2);
        legend_str=strvcat(legend_str,'GPS (lcm)');
    elseif (isfield (nav_t, 'GPSD3'))
        plot(nav_t.GPSD3.longitude.*RTOD, nav_t.GPSD3.latitude.*RTOD,'r.','LineWidth',2);
        legend_str=strvcat(legend_str,'GPS (lcm)');
    end
end

hold off;

legend_str(1,:)=[]; legend(legend_str); 
title('Position XY Lat/Lon plot');
xlabel('X  [decimal degrees]');
ylabel('Y  [decimal degrees]');
axis equal;
