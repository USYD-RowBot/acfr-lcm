function xygpsvalidplt(checkbox)
  
cla reset;

% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

% get good and bad gps for plotting
threshold = 4;
index_good = find(iver_t.gps.Num_Sats >= threshold);
index_bad  = find(iver_t.gps.Num_Sats <  threshold);

plot(	iver_t.x(index_good), iver_t.y(index_good), 'b.',...
		iver_t.x(index_bad), iver_t.y(index_bad), 'r.')

% get extents for text
xextents = get(gca,'XLim');
yextents = get(gca,'YLim');
xrange = xextents(2) - xextents(1);
yrange = yextents(2) - yextents(1);
text(xextents(1)+0.05*xrange, yextents(1)+0.0375*yrange, sprintf('Blue: Valid GPS fix\nRed:  Invalid GPS fix'));

title('Position XY plot (with valid gps)');
xlabel('X pos');
ylabel('Y pos');
axis equal;
% adding an argument to check goal position
if(checkbox==1)
    xygoalptplt;
end
