function viotool(axis_limits)
%VIOTOOL design tool for percent temporal overlap.
%  VIOTOOL displays a figure of percent temporal overlap as a
%  function of image period and forward speed at a given altitude.  The
%  slider bar allows the users to change the altitude.
%
%  VIOTOOL(LIMITS) allows the users to override the default axis
%  limits.
%
% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
%2004-01-29    Ryan Eustice              Created & written.
%2008-03-26    RME                       Renamed from cam_geo_nodim.m to viotool.m

DTOR = pi/180;
RTOD = 180/pi;

min_alt = 0.5;
max_alt = 5;

if ~exist('axis_limits','var')
  axis_limits = [0 10 0 2];
end

% CAMERA FOV
FOVx = 42.4*DTOR; % x axis across tracklines
FOVy = 34.5*DTOR; % y axis along direction of motion

% non-dim image footprint (normalized by altitude)
H_prime = 2*tan(FOVy/2);
W_prime = 2*tan(FOVx/2);

speed_prime_vec = [axis_limits(1):0.01:axis_limits(2)]';

Tp_vec = [axis_limits(3):0.01:axis_limits(4)]'; % flash period [s]

% calculate non-dim percent temporal overlap
pT_mat = 1 - Tp_vec * speed_prime_vec' / H_prime;

cmap = hsv;
cvec = [0:0.1:1]';
% plot percent temporal overlap as a function of flash rate and
% normalized speed
figure(1); clf;
[cmat,h.contours] = contour(speed_prime_vec,Tp_vec,pT_mat,cvec);
h.clabel = clabel(cmat,h.contours);
axis square;
grid on;
colormap(cmap);
axis(axis_limits);
xlabel('Fwd speed / Altitude');
ylabel('flash period [s]');
title('Percent Temporal Overlap');

% plot percent temporal overlap as a function of flash rate and fwd speed
% for a given altitude
figure(2); clf;
[cmat,h.contours] = contour(speed_prime_vec,Tp_vec,pT_mat,cvec);
h.clabel = clabel(cmat,h.contours);
for ii=1:length(h.contours)
  set(h.contours(ii),'UserData',get(h.contours(ii),'Xdata'));
end
for ii=1:length(h.clabel)
  set(h.clabel(ii),'UserData',get(h.clabel(ii),'Position'));
end
h.axis = gca;
set(h.axis,'UserData',get(h.axis,'XLim'));
set(h.axis,'Units','Normalized');
pos = get(h.axis,'Position');
h.slider = uicontrol(gcf, ...
		     'Style','slider', ...
		     'Min',min_alt,'Max',max_alt, ...
		     'SliderStep',[1 0.1/0.25]*0.25/(max_alt-min_alt), ...
		     'Value',1, ...
		     'Units','Normalized', ...
		     'Position',[pos(1),pos(2)-0.1,pos(3),0.045], ...
		     'Callback',@slider_callback);
h.text = uicontrol(gcf, ...
		   'Style','text', ...
		   'String',get(h.slider,'Value'), ...
		   'Units','Normalized', ...
		   'Position',[pos(1)-0.1,pos(2)-0.1,0.1,0.05]);
set(h.slider,'UserData',h);
axis square;
colormap(cmap);
axis(axis_limits);
grid on;
xlabel('Fwd speed [m/s]');
ylabel('flash period [s]');
title('Percent Temporal Overlap For Given Altitude');


function slider_callback(hObject, eventdata, handles)
% hObject       handle to slider
% eventdata     reserved
% handles       structure with handles and user data
h = get(hObject,'UserData');
altitude = get(h.slider,'Value');
for ii=1:length(h.clabel)
    set(h.clabel(ii),'Position',get(h.clabel(ii),'UserData').*[altitude,1,1]);
end
for ii=1:length(h.contours)
    %set(h.contours(ii),'XData',get(h.contours(ii),'UserData')*altitude);
end
set(h.axis,'XLim',get(h.axis,'UserData')*altitude);
set(h.text,'String',altitude);
