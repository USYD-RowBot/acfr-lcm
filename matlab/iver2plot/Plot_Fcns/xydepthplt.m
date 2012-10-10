function xydepthplt(checkbox)
  
cla reset;

% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

XY_CLIM = [min(iver_t.z) max(iver_t.z)];
setclim(XY_CLIM);

iver_t.z(iver_t.z<XY_CLIM(1)) = XY_CLIM(1);
iver_t.z(iver_t.z>XY_CLIM(2)) = XY_CLIM(2);
  
fscatter3(iver_t.x,iver_t.y,iver_t.z,iver_t.z,jet);
caxis(XY_CLIM);
ch = colorbar('horiz');

ylabel(ch,'Depth [m]');

title('Position XY plot (with depth)');
xlabel('X pos');
ylabel('Y pos');
axis equal;
% adding an argument to check goal position
if(checkbox==1)
    xygoalptplt;
end
