function headingplt()
% heading plot
% read from:
% 1) magnetic heading
% 2) os compass
% 3) gps
% 4) kvh
% 5) microstrain

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    rme         Created and written.
%    2008.07.17      ar          Modified for iver plot
%    2009.06.20      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match

cla reset;
legend_str=''; 
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
end

if IVER
    gps_hdg = iver_t.gps.True_Heading;
    gps_hdg(iver_t.gps.Num_Sats == 0) = NaN;
    
    plot(iver_t.elaptime, iver_t.comp.Magnetic_Heading,'b',...
         iver_t.elaptime, iver_t.comp.True_Heading, 'g',...
         iver_t.elaptime, gps_hdg, 'r','LineWidth',2);
    legend_str=strvcat(legend_str,'mag (uvc)','true (uvc)','gps (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    if (nav_t.KVH.mode == 3) % angle mode
        plot(nav_t.KVH.elaptime, nav_t.KVH.data);
        legend_str=strvcat(legend_str,'KVH (lcm)');
    end
    % depending on type of Microstrain in log...
    if (isfield(nav_t.MICROSTRAIN,'sEuler'))
        microstrain_data = nav_t.MICROSTRAIN.sEuler(:,3)*RTOD+180;
    elseif (isfield(nav_t.MICROSTRAIN,'Euler'))
        microstrain_data = nav_t.MICROSTRAIN.Euler(:,3)*RTOD+180;
    end
        
    plot(nav_t.MICROSTRAIN.elaptime, microstrain_data,'c',...
         nav_t.OS_COMPASS.elaptime, nav_t.OS_COMPASS.rph(:,3)*RTOD,'k');
    legend_str=strvcat(legend_str,'ms (lcm)','os-comp (lcm)');
end

hold off
title('Heading vs Time');
ylabel('Heading [degrees]');
legend(legend_str);
