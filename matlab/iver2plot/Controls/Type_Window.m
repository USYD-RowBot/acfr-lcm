function Type_Window()
    
% 2005-12-19    mvj    added WT vel vs. time plot button


% initalize the plot type window
figtype = figure(3);  clf;
set(figtype,'Name','Plot Type', ...
	    'Menubar','none', ...
	    'NumberTitle','off', ...
	    'Resize','off', ...
	    'Units','pixels', ...
	    'Position',[768 185 205 580], ...
	    'Tag','figtype');
	      
% setup default button size
NP = 29;                    % number of the button
LX1 = 1/(NP+1);
pl1 = zeros(NP,4);
plthandle = zeros(NP,1);

for i = 1:NP,
  pl1(i,:) = [0.1, 1.0-(i+1)*LX1, 0.8, 0.8*LX1];
end

np = 0;
% create XY plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','XY Plot', ...
	  'Units','normalized', ...
	  'Callback','PositionPlot(''xyplt'');', ...
	  'Position',pl1(np,:));

% create XY Lat/Lon plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','XY Lat/Lon', ...
	  'Units','normalized', ...
	  'Callback','PositionPlot(''xylatlonplt'');', ...
	  'Position',pl1(np,:));

% create X plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','X Plot', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''xplt'');', ...
	  'Position',pl1(np,:));

% create Y plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Y Plot', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''yplt'');', ...
	  'Position',pl1(np,:));

% create Depth plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Depth', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''depthplt'');', ...
	  'Position',pl1(np,:));

% create Altitude plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Seafloor', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''altitudeplt'');', ...
	  'Position',pl1(np,:));

% create Altimeter plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Altimeter', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''altimplt'');', ...
	  'Position',pl1(np,:));

% create Heading plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Heading', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''headingplt'');', ...
	  'Position',pl1(np,:));

% create pitch plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Pitch', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pitchplt'');', ...
	  'Position',pl1(np,:));

  % create roll plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Roll', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''rollplt'');', ...
	  'Position',pl1(np,:));

% create Fin Angles plot button (Pitch)
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Ctrl Surf Pitch', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''servoangleplt_pitch'');', ...
	  'Position',pl1(np,:));

% create Fin Angles plot button (Roll)
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Ctrl Surf Yaw', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''servoangleplt_roll'');', ...
	  'Position',pl1(np,:));
  
% create depth reference plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Depth Goal Estimate', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''dgoalplt_estimated'');', ...
	  'Position',pl1(np,:));

% create depth controller error terms plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','_err terms_', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''err_terms'');', ...
	  'Position',pl1(np,:));

% create depth controller plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Pitch Ref Estimate', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pitchrefplt_estimated'');', ...
	  'Position',pl1(np,:));

% create Motor Speed plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Motor Reference', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''motorspeedplt'');', ...
	  'Position',pl1(np,:));

% create Speed plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Speed', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''speedplt'');', ...
	  'Position',pl1(np,:));

% create numsats plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','GPS Satellite Count', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''numsatsplt'');', ...
	  'Position',pl1(np,:));

% create  Water Temperature plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Water Temperature', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''temp_water_plt'');', ...
	  'Position',pl1(np,:));

% create Internal Temperature plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Tube Temperature', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''temp_internal_plt'');', ...
	  'Position',pl1(np,:));  

% create Battery % plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Battery Level', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_batt_perc'');', ...
	  'Position',pl1(np,:));

% create Battery time remaining plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Battery Time Remaining', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_batt_time_left'');', ...
	  'Position',pl1(np,:));

% create Watt-hours plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Watt-hours', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_watt_hours'');', ...
	  'Position',pl1(np,:));

% create Power (watts) plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Power (watts)', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_power_watts'');', ...
	  'Position',pl1(np,:));

% create Battery voltage plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Battery Voltage', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_batt_volts'');', ...
	  'Position',pl1(np,:));

% create Battery Amps plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Battery Current', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''pow_batt_amps'');', ...
	  'Position',pl1(np,:));

% create time diff plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','diff(time)', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''difftimeplt'');', ...
	  'Position',pl1(np,:));

% create current step plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Waypt #', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''current_step'');', ...
	  'Position',pl1(np,:));

% create current camera frame plot button
np = np+1;
uicontrol('Parent',figtype, ...
	  'Style','Push', ...
	  'String','Camera Data', ...
	  'Units','normalized', ...
	  'Callback','TimePlot(''camera_imgnum_plt'');', ...
	  'Position',pl1(np,:));
  


