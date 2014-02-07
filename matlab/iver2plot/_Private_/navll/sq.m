function sq
xx = axis;
dx = max( [xx(2)-xx(1);xx(4)-xx(3)])/2;
xc = (xx(1) + xx(2))/2;
yc = (xx(3) + xx(4))/2;

axis([xc-dx,xc+dx,yc-dx,yc+dx]);
axis('square');
