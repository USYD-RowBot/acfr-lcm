function plot_epipolar_lines(lmat,dim)
%PLOT_EPIPOLAR_LINES 
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-06-2002      rme         Created and written.
%    12-13-2002      rme         Center of upper left pixel is defined
%                                to be (0,0) not (0.5,0.5)

nr = dim(1);
nc = dim(2);

for ii=1:size(lmat,2)
  [xx,yy] = line_bounds(0,nc-1,0,nr-1,lmat(:,ii));
  line(xx,yy,'color','g','linewidth',1.25);
end
