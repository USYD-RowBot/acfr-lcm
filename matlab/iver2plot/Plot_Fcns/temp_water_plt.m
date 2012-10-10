function temp_plt()
%temp plot

% read from:
% 1) ocean server compass
% 2) desert star

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    rme         Created and written.
%    2008.07.17      ar          Modified for iver plot
%    2009.06.17      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match


cla reset;
legend_str=''; 
hold on;


% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
    errordlg('This plot requires lcmlog data','Error');
    return;
end

if NAV
    plot(nav_t.DESERT_STAR.elaptime, nav_t.DESERT_STAR.temperature,'r','LineWidth',2);
    plot(nav_t.RDI.elaptime, nav_t.RDI.xducer_head_temp,'c.','LineWidth',2);
    legend_str=strvcat(legend_str,'dstar (lcm)','RDI (lcm)');
end

hold off;
title('Temperature vs Time');
ylabel('Temperature [C]');
legend(legend_str);
