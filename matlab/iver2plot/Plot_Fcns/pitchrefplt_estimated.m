function pitchrefplt_estimated()

cla reset;
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    hold off;
    return;
end

if IVER
    est = estimate_control();

    plot(iver_t.elaptime,iver_t.ref.Pitch_K_Goal, ...
         iver_t.elaptime,est.pitch_est,'LineWidth',2);
end

hold off;
ylabel('pitch reference');
title('depth control');
legend('p k goal (uvc)','p est (uvc)')
