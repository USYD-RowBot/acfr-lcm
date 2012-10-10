xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');
dx = xlim(2)-xlim(1);
dy = ylim(2)-ylim(1);
x0 = xlim(1) + .05*dx;
y0 = ylim(1) + .35*dy;
%set(gca,'Clipping','Off');
hold on
for i = 1:max(series)
   plot(x0,y0,[colors(i),symbols(i)]);
   text(x0+.015*dx,y0,sprintf('%4d',years(i)));
   y0 = y0-.05*dy;                 
end
hold off
