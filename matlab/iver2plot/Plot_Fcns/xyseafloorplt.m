function xyseafloorplt(checkbox)
  
cla reset;
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

XY_CLIM = [min(iver_t.seafloor) max(iver_t.seafloor)];
setclim(XY_CLIM);

iver_t.seafloor(iver_t.seafloor<XY_CLIM(1)) = XY_CLIM(1);
iver_t.seafloor(iver_t.seafloor>XY_CLIM(2)) = XY_CLIM(2);
  
fscatter3(iver_t.x,iver_t.y,iver_t.seafloor,iver_t.seafloor,jet);
caxis(XY_CLIM);
ch = colorbar('horiz');

ylabel(ch,'Seafloor [m]');

title('Position XY plot (with seafloor)');
xlabel('X pos');
ylabel('Y pos');
axis equal;
% adding an argument to check goal position
if(checkbox==1)
    xygoalptplt;
end
