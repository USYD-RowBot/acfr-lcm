function xyplt
cla reset;
legend_str=''; 
hold on

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
    plot(iver_t.x, iver_t.y, 'g-','LineWidth',2);
    plot(iver_t.x, iver_t.y, 'b.', 'MarkerSize', 6);
    plot(iver_t.x(1), iver_t.y(1), 'g*', 'MarkerSize', 6);
    plot(iver_t.x(end), iver_t.y(end), 'r*', 'MarkerSize', 6);
    legend_str=strvcat(legend_str,'pos (uvc)','pos (uvc)', 'start', 'end');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

hold off;
title('Position XY plot');
xlabel('X pos');
ylabel('Y pos');
legend(legend_str); 
axis equal;

dcm_obj = datacursormode(gcf);
set(dcm_obj, 'UpdateFcn', @myupdatefcn, 'Enable', 'on');

%==========================================================================
function txt = myupdatefcn(empt, event_obj)
	iver_t = evalin('base', 'iver_t');

	pos = get(event_obj, 'Position');
	x = pos(1);
	y = pos(2);
    
    axis equal;

	xind = find(iver_t.x == x);
	yind = find(iver_t.y == y);
	ind = min(xind, yind);

	txt = {	['X:    ',num2str(iver_t.x(xind))],...
			['Y:    ',num2str(iver_t.y(yind))],...
			['Time: ',num2str(iver_t.elapsed(ind))],...
			['Anim: ',num2str(iver_t.ref.Speed_CMD(ind))]};
