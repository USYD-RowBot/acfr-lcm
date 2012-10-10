function depthplt()
% pressure plot
% read from:
% 1) desert start

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2009.06.17      ak          Created and written.

legend_str='                                                                 '; 
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
catch
    errordlg('This plot requires lcmlog data','Error');
    hold off;
    return;
end

if exist('nav_t','var')
    plot(nav_t.desertstar.elapsed, nav_t.desertstar.p_abs,'b','LineWidth',2);
    plot(nav_t.desertstar.elapsed, nav_t.desertstar.p_gage,'r','LineWidth',2);
    plot(nav_t.desertstar.elapsed, nav_t.desertstar.p_atm,'g','LineWidth',2);    
    
    legend_str=strvcat(legend_str,'p_{abs}','p_{gage}','p_{atm}');
end
hold off;

% make title, ylabel and legend
title('Pressure vs Time');
ylabel('Pressure [Pa]');
legend_str(1,:)=[]; legend(legend_str);  
