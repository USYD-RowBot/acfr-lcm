function xydvlplt

[a,b,c,d] = legend
legend_str = strvcat(d{:});
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    return;
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    gpsfield = [];
    if (isfield (nav_t, 'GPSD'))
        gpsfield = 'GPSD';
    elseif (isfield (nav_t, 'GPSD3'))
        gpsfield = 'GPSD3';
    end
    [xgps,ygps] = ll2xy (nav_t.(gpsfield).latitude*RTOD, nav_t.(gpsfield).longitude*RTOD, iver_t.ORGLAT, iver_t.ORGLON);
    plot(xgps, ygps, 'mo', 'MarkerSize', 4);
    plot(nav_t.DVLRENAV.nx, nav_t.DVLRENAV.ny, 'c.-', 'MarkerSize', 4,'LineWidth', 2);
    legend_str=strvcat(legend_str,'gpsd (lcm)','dvlrenav (lcm)');
end

hold off;
legend(legend_str); 
