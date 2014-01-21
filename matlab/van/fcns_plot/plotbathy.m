function plotbathy(udata,vdata,Image,bathy_t)
%function plotbathy(udata,vdata,Image,bathy_t)

%subplot(211);
ax1 = axes;
[h,sel] = fscatter3(bathy_t.X,bathy_t.Y,bathy_t.Z,-bathy_t.Z,jet);
axis image ij; grid on; set(gca,'ZDir','reverse');
xlabel('x_c'); ylabel('y_c'); zlabel('z_c');
%title('XYZ bathy points in camera frame');

%subplot(212);
ax2 = axes;
subimagesc(udata,vdata,Image.Iwarp); colormap gray; axis image off;
hold on;
for k=1:length(h)
  plot(bathy_t.uc(sel{k}),bathy_t.vc(sel{k}),'+','Color',get(h(k),'Color'),'linewidth',1.75);
end
delete(ax1);
hold off;
title('RDI measured scene depth');
cb2 = colorbar;
poscb2 = get(cb2,'position');
posax2 = get(ax2,'position');
delete(cb2);

ax3 = axes;
clim = [min(bathy_t.Z),max(bathy_t.Z)];
caxis(clim);
colormap(flipud(jet));
cb3 = colorbar; 
set(cb3,'Position',poscb2);
set(ax3,'visible','off');
set(ax2,'Position',posax2);
