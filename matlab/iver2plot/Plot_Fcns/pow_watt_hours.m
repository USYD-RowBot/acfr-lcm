function pow_watt_hours()

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
    plot(iver_t.elaptime, iver_t.pow.Watt_Hours,'LineWidth',2);
end

hold off;
ylabel('Watt-hours');
title('Watt-hours');
legend('uvc');
