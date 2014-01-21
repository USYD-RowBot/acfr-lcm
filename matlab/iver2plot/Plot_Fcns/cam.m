function [] = cam()
%----------------------------------------------------------------------
%Michelle Howard
%June 30th 2011
%This function plots the XY trajectory of the Iver (blue) and the camera
%footprints (red)
%-----------------------------------------------------------------------

%cla reset;
legend_str=''; 
hold on

% uvclog------------------------------------------------------------------
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch  %#ok<*CTCH>
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    return;
end

try
    nav_t = evalin('base','nav_t');
    IVER=1;
catch 
    IVER=0;
    errordlg('I don''t know whats wrong','Error');
    return;
end

try
    nav_t = evalin('base','nav_t');
    NAV=1; %#ok<*NASGU>
catch
    NAV=0;
    errordlg('This plot requires lcmlog data','Error');
    return;
end

try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
end
%--------------------------------------------------------------------------

% time photos were taken
if isfield (nav_t, 'PROSILICA_M')
    t_imgs = nav_t.PROSILICA_M.unixtime;
elseif isfield (nav_t, 'PROSILICA_C')
    t_imgs = nav_t.PROSILICA_M.unixtime;
else
    errordlg ('This plot requires camera data', 'Error');
    return;
end


x_imgs = interp1 (iver_t.unixtime, iver_t.x, t_imgs); % X Coordinates of Images
y_imgs = interp1 (iver_t.unixtime, iver_t.y, t_imgs); % Y Coordinates of Images
%plot(iver_t.x, iver_t.y, 'b', x_imgs, y_imgs, 'r');  % Plots XY locations of Pictures


centerX  = roll(x_imgs, iver_t, t_imgs);          % Adjusts X Coordinates according to roll
centerY  = pitch(y_imgs, iver_t, t_imgs);         % Adjusts Y Coordinates according to pitch
points   = yaw(centerX, centerY, iver_t,t_imgs);  % Adjusts XY Coordinates according to yaw
points1X = points(:,1);                           % Matrix of X Coordinates of the bottom left corner of the field of view
points1Y = points(:,2);                           % Matrix of Y Coordinates of the bottom left corner of the field of view
points2X = points(:,3);                           % Matrix of X Coordinates of the bottom right corner of the field of view
points2Y = points(:,4);                           % Matrix of Y Coordinates of the bottom right corner of the field of view
points3X = points(:,5);                           % Matrix of X Coordinates of the top right corner of the field of view
points3Y = points(:,6);                           % Matrix of Y Coordinates of the top right corner of the field of view            
points4X = points(:,7);                           % Matrix of X Coordinates of the top left corner of the filed of view
points4Y = points(:,8);                           % Matrix of Y Coordinates of the top left corner of the filed of view

MatrixX = [points1X';points2X';points3X';points4X';points1X'];
MatrixY = [points1Y';points2Y';points3Y';points4Y';points1Y'];
plot(MatrixX, MatrixY,'-r');

hold on;
axis equal;

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

hold off;
title('Position XY plot');
xlabel('X pos');
ylabel('Y pos');
%legend(legend_str); 
axis equal;

dcm_obj = datacursormode(gcf);
set(dcm_obj, 'UpdateFcn', @myupdatefcn, 'Enable', 'on');
end

%==========================================================================
function txt = myupdatefcn(empt, event_obj) %#ok<INUSL>
	iver_t = evalin('base', 'iver_t');

	pos = get(event_obj, 'Position');
	x = pos(1);
	y = pos(2);
    
    axis equal;

	xind = find(iver_t.x == x);
	yind = find(iver_t.y == y);
	ind = min(xind, yind);

	txt = {	['X:    ',num2str(iver_t.x(xind))],...
			['Y:    ',num2str(iver_t.y(yind))],...
			['Time: ',num2str(iver_t.elapsed(ind))],...
			['Anim: ',num2str(iver_t.ref.Speed_CMD(ind))]};
end
