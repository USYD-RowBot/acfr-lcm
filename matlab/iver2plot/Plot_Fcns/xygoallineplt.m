function xygoallineplt()

plthdl = subplot_handle;
if isempty(plthdl)
  return;
end

% draw the goal points on the plot
ref = evalin('base','iver_t.ref');
gps = evalin('base','iver_t.gps');
xref = evalin('base','iver_t.xref');
yref = evalin('base','iver_t.yref');

[xref, yref] = collapse(xref, yref);
axis equal;
hold on;
plot(xref,yref,'g','LineWidth',2)
for i = 1:length(xref)
    text(xref(i)-.9,yref(i)-1.1,sprintf('%d',i),'FontSize',8,'FontWeight','bold')
end
hold off;
end

