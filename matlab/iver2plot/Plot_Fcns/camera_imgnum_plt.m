function camera_imgnum_plt()

cla reset;
legend_str=''; 
hold on;


% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
    errordlg('This plot requires lcmlog data','Error');
    return;
end

if NAV && isfield(nav_t,'PROSILICA_M')
    plot(nav_t.PROSILICA_M.elaptime, nav_t.PROSILICA_M.frame,'ro','LineWidth',2);
    legend_str=strvcat(legend_str,'PROSILICA\_M');
end
if NAV && isfield(nav_t,'PROSILICA_C')
    plot(nav_t.PROSILICA_C.elaptime, nav_t.PROSILICA_C.frame,'b.','LineWidth',2);
    legend_str=strvcat(legend_str,'PROSILICA\_C');
end

hold off;

title('Camera Frame # vs Time');
ylabel('Frame #');

if length(legend_str) > 0
    legend(legend_str);
else
    errordlg('No camera data', 'Error');
end
