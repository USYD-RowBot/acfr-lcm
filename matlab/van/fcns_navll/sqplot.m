function sqplot(x,y)

minx = min(x) ;
miny = min(y) ;
maxx = max(x);
maxy = max(y);
dx1 = maxx-minx ;
dy1 = maxy-min(y) ;
dx = dx1 ;
if(dy1 > dx1)
	dx = dy1 ;
end

xc = (minx + maxx)/2 ;
yc = (miny + maxy)/2 ;
dx2 = .6*dx ;
axis([xc-dx2, xc+dx2, yc-dx2, yc + dx2]) ;
axis('square');
