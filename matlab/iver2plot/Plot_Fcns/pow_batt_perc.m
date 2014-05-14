function pow_batt_perc()

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
    plot_margin = 5;
    plot(iver_t.elaptime, iver_t.pow.Batt_Percent,'LineWidth',2);

    min_bc = min(iver_t.pow.Batt_Percent) - plot_margin;
    max_bc = max(iver_t.pow.Batt_Percent) + plot_margin;
    if (min_bc < 0)
        min_bc = 0;
    end
    if (max_bc > 100)
        max_bc = 100;
    end
    ylim([min_bc, max_bc]);
end

hold off;
ylabel('%');
title('Battery Level');
legend('uvc');
