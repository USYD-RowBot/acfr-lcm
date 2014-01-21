function numsats()
%number of satellites plot

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
    plot(iver_t.elaptime, iver_t.gps.Num_Sats,'*','MarkerSize',4);
end

hold off;
title('GPS Satellite Count vs. Time');
legend('nsats (uvc)');
