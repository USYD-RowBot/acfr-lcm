function xygoalptplt()

plthdl = subplot_handle;
if isempty(plthdl)
  return;
end
% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

% draw the goal points on the plot
ref = iver_t.ref;
gps = iver_t.gps;
xref = iver_t.xref;
yref = iver_t.yref;

[xref, yref] = collapse(xref, yref);

hold on;
plot(xref,yref,'s','MarkerSize',12,'MarkerFaceColor','g','MarkerEdgeColor','k')
for i = 1:length(xref)
    text(xref(i)-.9,yref(i)-1.1,sprintf('%d',i),'FontSize',8,'FontWeight','bold')
end
hold off;
end
