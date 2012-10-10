function pow_batt_time_left()

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
    plot(iver_t.elaptime, iver_t.pow.Time_to_Empty,'LineWidth',2);
end

hold off;
ylabel('Time (Minutes)');
title('Battery Time Remaining');
legend('uvc');
