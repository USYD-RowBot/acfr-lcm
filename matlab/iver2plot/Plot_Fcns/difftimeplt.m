function difftimeplt()

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
    tmp = [1 diff(iver_t.elaptime')];
    tmp(2) = 1;
    tmp(end)=1;
    plot(iver_t.elaptime, tmp,'LineWidth',2);
end

hold off;
title('diff(time)');
ylabel('Time difference between samples (s)');
legend('uvc');
