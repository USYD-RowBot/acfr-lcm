%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% function bin_plot_xyz(x,y,z,cmap_string)
%
% plot a color coded dot cloud given points in x y z
% optional: cmap_string, like 'jet' or 'hot'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function bin_plot_xyz(x,y,z,cmap_string)


%binplot(x,y,z)
if(nargin == 4)
  eval(sprintf('cmap = %s(64);',cmap_string));
else
  cmap = jet(64);
end

[x,y,z] = denan(x,y,z);
zb = linspace(min(z),max(z),length(cmap));
[bins,occ_ptr] = binsort(z,zb,x,y);
for n = 1:length(occ_ptr)
  nn = occ_ptr(n);
  line(bins{nn}(:,2),bins{nn}(:,3),bins{nn}(:,1),...
  'color',cmap(nn,:),'linestyle','none','marker','.','markersize',10);
end
hold off

h = colorbar;
ch = get(h,'children');
set(ch,'ydata',zb([1 end]));
set(h,'ylim',zb([1 end]));

xlabel('x')
ylabel('y')
zlabel('z')

