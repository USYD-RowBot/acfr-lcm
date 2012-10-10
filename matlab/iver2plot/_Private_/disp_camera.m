function disp_camera
persistent kk ptrhdl;
if isempty(kk); kk=0; end

% delete old camera text box
delete(findobj('Tag','camera_text'));

kk = kk+1;
plthdl = subplot_handle;
if isempty(plthdl) || get(plthdl.label_checkbox,'Value')==0
  set(1,'WindowButtonMotionFcn',[]);
  return; % no plot
end

if ~mod(kk,10)
  return; % cut down on cpu usage
end

if isempty(ptrhdl) || ~ishandle(ptrhdl) % create
  hold on;
  ptrhdl = plot(nan,nan,'rx','markersize',8,'linewidth',2);
  hold off;
end

pos  = get(gca,'CurrentPoint');
POS  = mean(pos);
POS  = round(POS*1000)/1000;
pct  = 0.20;
FPOS = get(1,'CurrentPoint') + pct*[diff(xlim), diff(ylim)];

if POS(3)==0.5
  disp(' Try the program only after you actually plot a figure!');
else
  % position in plot
  PXF = evalin('base','nav_t.PXF');
  ii = dsearchn(PXF.XYZ(:,1:2),PXF.tes,POS(1:2));
  
  % compose lat/lon string
  imgnum = PXF.imgname(ii,end-7:end-4);

  % update text box
  uicontrol(1,'pos',[FPOS,40,15], ...
	    'style','text', ...
	    'string',imgnum, ...
	    'backgroundcolor',[0 0 0], ...
	    'foregroundcolor',[1 0 0], ...
	    'Interruptible','on', ...
	    'Tag','camera_text', ...
	    'ButtonDownFcn','delete(gcbo)');
  
  % indicate the image location on the screen
  set(ptrhdl,'xdata',PXF.XYZ(ii,1),'ydata',PXF.XYZ(ii,2));
end
