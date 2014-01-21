function pow_power_amps()

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
    plot(iver_t.elaptime, iver_t.pow.Batt_Ampers,'LineWidth',2);
end

hold off;
ylabel('Amps');
title('Current Draw');
legend('uvc');
