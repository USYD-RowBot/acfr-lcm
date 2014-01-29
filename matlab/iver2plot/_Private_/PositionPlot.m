function PositionPlot(plotfun)

user = get(1,'UserData');
ctlhdl = get(2,'UserData');

try
    iver_t = evalin('base','iver_t');
    IVERLOG=1;
catch
    IVERLOG=0;
end

if(IVERLOG)
    STARTTIME = evalin('base','iver_t.STARTTIME');
else
    STARTTIME = evalin('base','nav_t.STARTTIME');
end

% check that the user has selected a mission file
if length(STARTTIME) == 0
  errordlg('Select a mission file first!','Error');
  return;
end
  
% don't put the xyplot in a subplot
set(ctlhdl.top_radiobutton,'Value',0);
set(ctlhdl.bottom_radiobutton,'Value',0);
set(ctlhdl.full_radiobutton,'Value',1);

SetSubPlot;
SetupXYControls(plotfun);
eval(plotfun);
SetPlotSize;
grid on;

user.PH_FULL = plotfun;
user.PH_TOP = '';
user.PH_BOTTOM = '';
set(1,'UserData',user);


%==========================================================================
function SetupXYControls(plotfun)

Width=25;
Height=85;

figplt = figure(1); clf;
Pos = get(figplt,'Position');
plotfuncmd = sprintf('eval(''%s'');',plotfun);

plthdl.subplot_frame = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-125) 15 105 250], ...
    'Style','frame', ...
    'Tag','subplot_frame_position');

plthdl.cmin_inputbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[1 1 1], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = get(1,''UserData'');', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 225 Height Width], ...
    'String','', ...
    'HorizontalAlignment','left', ...
    'Style','edit', ...
    'Tag','clim_inputbox');
set(1,'WindowButtonMotionFcn',[]);

plthdl.dvlrenav_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.dvlrenav_checkbox,''Value'')==1);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',get(plthdl.goalpt_checkbox,''Value''));', ...
       'set(plthdl.goalline_checkbox,''Value'',get(plthdl.goalline_checkbox,''Value''));', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xydvlplt;', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 200 Height Width], ...
    'String','DVLRenav', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','dvlrenav_checkbox');

plthdl.depth_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.depth_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',get(plthdl.goalpt_checkbox,''Value''));', ...
       'set(plthdl.goalline_checkbox,''Value'',get(plthdl.goalline_checkbox,''Value''));', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xydepthplt(get(plthdl.goalpt_checkbox,''Value''));', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 175 Height Width], ...
    'String','Depth', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','depth_checkbox');

plthdl.gpsvalid_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.gpsvalid_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',get(plthdl.goalpt_checkbox,''Value''));', ...
       'set(plthdl.goalline_checkbox,''Value'',get(plthdl.goalline_checkbox,''Value''));', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xygpsvalidplt(get(plthdl.goalpt_checkbox,''Value''));', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 150 Height Width], ...
    'String','GPSvalid', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','gpsvalid_checkbox');

plthdl.seafloor_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.seafloor_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',get(plthdl.goalpt_checkbox,''Value''));', ...
       'set(plthdl.goalline_checkbox,''Value'',get(plthdl.goalline_checkbox,''Value''));', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xyseafloorplt(get(plthdl.goalpt_checkbox,''Value''));', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 125 Height Width], ...
    'String','Seafloor', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','seafloor_checkbox');

plthdl.heading_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.heading_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',get(plthdl.goalpt_checkbox,''Value''));', ...
       'set(plthdl.goalline_checkbox,''Value'',get(plthdl.goalline_checkbox,''Value''));', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xyheadingplt(get(plthdl.goalpt_checkbox,''Value''));', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 100 Height Width], ...
    'String','Heading', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','heading_checkbox');

plthdl.goalpt_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.goalpt_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalline_checkbox,''Value'',0);', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xygoalptplt;', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 75 Height Width], ...
    'String','GoalPts', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','goalpt_checkbox');

plthdl.goalline_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.goalline_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',0);', ...
       'set(plthdl.camfootprint_checkbox,''Value'',0);'...
       'drawnow;', ...
       'xygoallineplt;', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 50 Height Width], ...
    'String','GoalLines', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','goalline_checkbox');

plthdl.camfootprint_checkbox = ...
    uicontrol('Parent',figplt, ...
    'Units','pixels', ...
    'BackgroundColor',[0.7 0.7 0.7], ...
    'Value',0, ...
    'Callback', ...
    ['plthdl = subplot_handle;', ...
    'axe = axis;', ...
    plotfuncmd, ...
    'axis(axe);', ...
    'set(1,''pointer'',''watch'');', ...
    'if (get(plthdl.camfootprint_checkbox,''Value'')==1);', ...
       'set(plthdl.dvlrenav_checkbox,''Value'',0);', ...
       'set(plthdl.depth_checkbox,''Value'',0);', ...
       'set(plthdl.gpsvalid_checkbox,''Value'',0);', ...
       'set(plthdl.seafloor_checkbox,''Value'',0);', ...
       'set(plthdl.heading_checkbox,''Value'',0);', ...
       'set(plthdl.goalpt_checkbox,''Value'',0);', ...
       'set(plthdl.goalline_checkbox,''Value'',0);'...
       'drawnow;', ...
       'cam;', ...
    'end;', ...
    'set(1,''pointer'',''arrow'');', ...
    'SetPlotSize;', ...
    'SetDrag;', ...
    'grid on;', ...
    'clear plthdl axe;', ...
    ], ...
    'ListboxTop',0, ...
    'Position',[(Pos(3)-115) 25 Height Width], ...
    'String','CamFOV', ...
    'HorizontalAlignment','left', ...
    'Style','checkbox', ...
    'Tag','camfootprint_checkbox');

set(plthdl.subplot_frame,'UserData',plthdl);
