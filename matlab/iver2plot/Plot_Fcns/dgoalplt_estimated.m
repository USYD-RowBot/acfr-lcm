function dgoalplt_estimated()

cla reset;
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

if IVER
    est = estimate_control();

    plot(iver_t.elaptime,est.dgoal*FTOM,'-b.', ...
         iver_t.elaptime,est.depth*FTOM,'g', ...
         iver_t.elaptime,est.depth_goal*FTOM,'r',...
         'LineWidth',2);
end

hold off;
ylabel('depth [m]');
title('depth goal comparison');
legend('estimated reference','depth','mission reference');
