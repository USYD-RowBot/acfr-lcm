function servoangleplt()

% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

plot(iver_t.elapsed, iver_t.ctrl.Pitch_L,...
	iver_t.elapsed, iver_t.ctrl.Pitch_R,...
    iver_t.elapsed, iver_t.ctrl.Yaw_T,...
    iver_t.elapsed, iver_t.ctrl.Yaw_B,'LineWidth',2);

title('Control Surfaces Pitch (P) / Yaw (Y)');
ylabel('Angle');
legend('P(left)', 'P(right)','Y(top)','Y(bottom)',-1) 
