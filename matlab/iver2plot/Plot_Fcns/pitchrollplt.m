function pitchrollplt()

% pitch and roll plot
% read from:
% 1) os compass
% 2) microstrain

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2005.12.20      rme         added OCTANS
%    2008.07.17      ar          Modified for iver plot
%    2009.06.20      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match

user = get (1,'UserData');
cla reset;
legend_str='                                                                 '; 
  
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
end
if exist('iver_t','var')
    plot(	iver_t.elapsed, iver_t.comp.Pitch_Angle,'b:',...
            iver_t.elapsed, iver_t.comp.Roll_Angle,'r:',...
            iver_t.elapsed, iver_t.ref.Pitch_K_Goal,'g:');

    legend_str=strvcat(legend_str,'p (uvc)','r (uvc)','p goal (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
end
if exist('nav_t','var')
     % depending on type of Microstrain in log...
    if (isfield(nav_t.MICROSTRAIN,'sEuler'))
        microstrain_data = nav_t.MICROSTRAIN.sEuler;
    elseif (isfield(nav_t.MICROSTRAIN,'Euler'))
        microstrain_data = nav_t.MICROSTRAIN.Euler;
    end

    if exist('iver_t','var')
        TIMEDIFF=user.TIMEDIFF;
        nav_t.MICROSTRAIN.mtime = nav_t.MICROSTRAIN.unixtime-iver_t.unixtime(1)-TIMEDIFF;% Match the time difference between unixtime
        nav_t.OS_COMPASS.mtime = nav_t.OS_COMPASS.unixtime-iver_t.unixtime(1)-TIMEDIFF;
        plot(nav_t.MICROSTRAIN.mtime, -microstrain_data(:,2)*RTOD,'r','LineWidth',2);
        plot(nav_t.OS_COMPASS.mtime, nav_t.OS_COMPASS.rph(:,2)*RTOD,'b','LineWidth',2)
        plot(nav_t.MICROSTRAIN.mtime, -microstrain_data(:,2)*RTOD,'g','LineWidth',2);
        plot(nav_t.OS_COMPASS.mtime, nav_t.OS_COMPASS.rph(:,2)*RTOD,'k','LineWidth',2);
    else
        plot(nav_t.MICROSTRAIN.elapsed*1E-6, -microstrain_data(:,2)*RTOD,'r','LineWidth',2);
        plot(nav_t.OS_COMPASS.elapsed*1E-6, nav_t.OS_COMPASS.rph(:,2)*RTOD,'b','LineWidth',2)
        plot(nav_t.MICROSTRAIN.elapsed*1E-6, -microstrain_data(:,1)*RTOD,'g','LineWidth',2);
        plot(nav_t.OS_COMPASS.elapsed*1E-6, nav_t.OS_COMPASS.rph(:,1)*RTOD,'k','LineWidth',2);
    end
    legend_str=strvcat(legend_str,'p ms (lcm)','p comp (lcm)','r ms (lcm)','r comp (lcm)');
end

hold off;

ylabel('Degrees');
title('Pitch and Roll vs Time');
legend_str(1,:)=[]; legend(legend_str);
