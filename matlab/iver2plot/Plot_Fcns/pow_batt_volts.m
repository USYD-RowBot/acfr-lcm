function pow_batt_volts()

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
    plot(iver_t.elaptime, iver_t.pow.Batt_Volts,'LineWidth',2);
end

hold off;
ylabel('Volts');
title('Battery Voltage');
legend('uvc');
