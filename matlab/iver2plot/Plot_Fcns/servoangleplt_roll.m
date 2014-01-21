function servoangleplt_roll()

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
    plot(iver_t.elaptime, iver_t.ctrl.Yaw_T,...
         iver_t.elaptime, iver_t.ctrl.Yaw_B,'LineWidth',2);
end
     
hold off;
title('Control Surfaces Yaw (Y)');
ylabel('Counts');
legend('Yaw\_T','Yaw\_B',-1) 
