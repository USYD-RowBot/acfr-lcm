function servoangleplt_pitch()

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
    plot(iver_t.elaptime, iver_t.ctrl.Pitch_L,...
         iver_t.elaptime, iver_t.ctrl.Pitch_R,'LineWidth',2);
end

hold off;
title('Control Surfaces Pitch (P)');
ylabel('Counts');
legend('Pitch\_L', 'Pitch\_R',-1) 
