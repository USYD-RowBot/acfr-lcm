function [] = iver2plot()
%IVER2PLOT graphical user interface for plotting iver2 log files
%  IVER2PLOT is based on SEABEDPLOT, originally written by Ryan Eustice.
%  Regarding SEABEDPLOT:
%    SEABEDPLOT is a GUI with controls for plotting and viewing .auv
%    files.  When SEABEDPLOT is called, it looks in the current directory
%    for valid .log,.syscfg, and .auv files.  The valid files are presented
%    to the user for display.  If none are found, the user is prompted to
%    type in a valid data directory.
%
%  History:
%  2008/07/01  Began SEABEDPLOT->IVER2PLOT conversion (Andrew Richardson)
%  2001/07/29  Written and created by Ryan Eustice.  
%              Loosely based upon a GUI written by Chris Roman,
%              which was in turn based upon an original plot program
%              written by Dana Yoerger.

%================================
% SET PATH
%================================
rootdir = which('iver2plot.m');
filenamesize = size('iver2plot.m',2);
rootdir = rootdir(1:end-filenamesize);
addpath(genpath(rootdir));
addpath(genpath([rootdir,'../van']));

%================================
% INITIALIZE IVER2PLOT USER STRUCT
%================================
user.rootdir = rootdir;

% assign UVC version number
user.UVC_VERSION = 4.7;

% create default time difference
user.TIMEDIFF=0;

% current configuration of figure
user.PH_FULL = '';
user.PH_TOP = '';
user.PH_BOTTOM = '';

%================================
% CREATE PLOT WINDOW
%================================
% initalize the plotting window
figure(1);  clf;
set(1,'Name','iver2 Plot', ...
      'NumberTitle','off', ...
      'Units','pixels', ...
      'Position',[18 215 735 522], ...
      'Resize','on', ...
      'Toolbar','none', ...
      'Tag','FIGPLOT', ...
      'UserData',user);

%================================
% CREATE PLOT TYPE WINDOW
%================================
Type_Window;

%================================
% CREATE PLOT CONTROL WINDOW
%================================
Control_Window;

%================================
% LOAD PREVIOUS WINDOW SETTINGS
%================================
posfile = [rootdir,'.iver2plot.dat'];
if exist(posfile,'file');
  pos = load(posfile);
  set(1,'Position',pos(1,:));
  set(2,'Position',pos(2,:));
  set(3,'Position',pos(3,:));
end;


%================================
% CHECK IF THE DIRECTORY WE ARE IN
% CONTAINS VALID FILES
%================================
ctlhdl = get(2,'UserData');
set(ctlhdl.dir_edittext,'String',pwd);
dir_callback(1);
