function xplt()
  
cla reset;
legend_str=''; 
hold on

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    return;
end

if IVER
    plot(iver_t.elaptime, iver_t.x,...
         iver_t.elaptime, iver_t.xref);
    legend_str=strvcat(legend_str,'pos (uvc)','ref (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end
 
hold off;
title(sprintf('Position X vs Time'));
ylabel('X pos') ;
legend(legend_str); 
