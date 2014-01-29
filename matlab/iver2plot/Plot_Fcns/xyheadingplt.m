function xyheadingplt(checkbox)

plthdl = subplot_handle;
if isempty(plthdl)
  return;
end

% uvclog
try
    iver_t = evalin('base','iver_t');
catch
    errordlg('This plot requires UVC log data','Error');
    return;
end

% plot heading vectors
% Heading arrows on a subset of the points displayed
% if would be nice to be able to select the start and end time as
% well as the number of arrows in a dialog box of some kind
heading = iver_t.comp.True_Heading;
elapsedtime = iver_t.elaptime;

vec_dir = -(heading*DTOR - pi/2);

hx = cos(vec_dir);
hy = sin(vec_dir);

axis equal;
hold on
quiver(iver_t.x,iver_t.y,hx,hy,0,'r');
plot(iver_t.x,iver_t.y,'r.');
hold off
title(sprintf('Position XY plot\nred arrow heading'));
% adding an argument to check goal position
if(checkbox==1)
    xygoalptplt;
end
