function motorspeedplt()

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
    plot(iver_t.elaptime, iver_t.ref.Speed_CMD,'LineWidth',2);
end

hold off;
title('Commanded Prop RPM (range: 0-255)');
legend('uvc');
