function motortorqueplt()
%animatics motor torque plot

% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

iverSpeedCMD = iver_t.ref.Speed_CMD;
animV = (iverSpeedCMD - 128) * (-10000);
animRevPSec = animV/32212;
animRPM = animRevPSec * 60;

plot(iver_t.elapsed, animRPM);
title('Commanded Motor Torque');
xlabel('Elapsed Time')
ylabel('animatics rpm')

