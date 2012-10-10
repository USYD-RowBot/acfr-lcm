function err_terms()

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
    est = estimate_control();

    plot(iver_t.elaptime,est.istate,...
         iver_t.elaptime,est.pstate,'LineWidth',2);
end

hold off;
legend('istate','pstate');
