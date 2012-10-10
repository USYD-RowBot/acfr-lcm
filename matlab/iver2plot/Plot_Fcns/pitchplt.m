function pitchplt()

% pitch plot
% read from:
% 1) os compass
% 2) microstrain

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2010.05.24      ak          Branched out from pitchrollplt

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
    plot(iver_t.elaptime, iver_t.comp.Pitch_Angle,'r',...
		 iver_t.elaptime, iver_t.ref.Pitch_K_Goal,'k','LineWidth',2);

    legend_str=strvcat(legend_str,'uvc','goal (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    % depending on type of Microstrain in log...
    if (isfield(nav_t.MICROSTRAIN,'sEuler'))
        microstrain_data = nav_t.MICROSTRAIN.sEuler(:,2)*RTOD;
    elseif (isfield(nav_t.MICROSTRAIN,'Euler'))
        microstrain_data = nav_t.MICROSTRAIN.Euler(:,2)*RTOD;
    end
    plot(nav_t.MICROSTRAIN.elaptime, -microstrain_data,'g','LineWidth',2);
    plot(nav_t.OS_COMPASS.elaptime, nav_t.OS_COMPASS.rph(:,2)*RTOD,'b','LineWidth',2)

    legend_str=strvcat(legend_str,'ms (lcm)','os (lcm)');
end

hold off;
ylabel('Degrees');
title('Pitch vs Time');
legend(legend_str);
