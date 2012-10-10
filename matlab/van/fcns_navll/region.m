figure(1)
xx = ginput(2);

xmin = xx(1,1);
ymin = xx(1,2);
xmax = xx(2,1);
ymax = xx(2,2);


ind = find( (xutm>xmin) & (yutm>ymin) & (xutm<xmax) & (yutm<ymax));
figure(2);
fprintf('found %d points\n',length(ind));
for i = 1:length(ind)
   k = ind(i);
   plot(xutm(k),yutm(k),[series_color(k),series_symbol(k)]);
   text(xutm(k),yutm(k),p(k).comment);
   hold on
end
hold off
utmlabel
mylegend
grid on
sq
