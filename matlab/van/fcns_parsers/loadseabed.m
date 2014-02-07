function nav_t = loadseabed(navdir,filename,NO_VALUE,RDIWORLDCFG);
%LOADSEABED  Loads ascii seabed data files.
%  NAV_T = LOADSEABED(NAVDIR,SYSCFG,NOVALUE,RDIWORLDCFG)
%  returns a NAV_T structure given the data directory, the name of the syscfg file,
%  NOVALUE, and RDIWORLDCFG.
%
%  NO_VALUE = -98765; % pre  12/2005
%  NO_VLAUE = -12345; % post 12/2005 (default)
%
%  RDIWORLDCFG is a flag that refers to the data contents of the
%  vx,vy,vz fields in the original RDI .RAW.auv log string.  This is
%  for legacy only, and is only required if parsing pre 12/2005 SeaBED
%  log files.
%    set to true  if vx,vy,vz represent world-frame velocities
%    set to false if vx,vy,vz represent u,v,w, *sensor-frame* velocities
%                 (this is the default if left unspecified)
%
%  EXAMPLE:
%  NAV_T = LOADSEABED('/files1/data/','20021202_1249.SEABED.syscfg',-12345,false)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-03-2002      rme         Created and written.
%    07-26-2003      rme         Modified to automatically determine
%                                which .CTL.auv and .RAW.auv are
%                                associated with argument syscfg.
%    07-29-2003      rme         Fixed a type in calculating time.unixtime
%    02-11-2004      rme         Added parsing of Phins heading
%    02-12-2004      rme         Added sanity check for existence of
%                                syscfg file
%    01-04-2006      rme         Added NOVALUE and RDILEGACY arguments
%    01-05-2006      rme         Added LBL travel time parsing
%    01-09-2006      rme         Added LBL parsing of .syscfg file
%    01-18-2006      rme         Added several sensors fields that were in
%                                seabed-plot, but not here.
%                                Reorganized LBL structure.
%    01-23-2006      rme         Added parsing of GPS & PPS.
%    01-24-2006      rme         Fixed a typo in PPS parsing format.
%                                Increased TOL in fix_no_value() from 0.001 to 0.1
%                                Updated RDIWORLDCFG to include pmfs2005 dataset.
%                                Added CATOA.
%                                Added TOPSIDEFLAG so loadtopside.m can use the 
%                                parsing code that already exists here.
%                                Added GYRO and LXT parsing.
%    04-24-2006      rme         Updated to use process_AUV script for grep processing
%    06-26-2006      rme         Added parsing of RDI depth field.
%                                Renamed abstime time field to unixtime.
%                                Added LBL.lbl_tat.
%    07-24-2006      rme         Added CATOA.tos time-of-send field and removed inlier ii field.

if ~argchk('NO_VALUE');
  NO_VALUE = -12345;
end;
if ~argchk('RDIWORLDCFG');
  RDIWORLDCFG = true;
end;

% automatically calculate the granularity of the waitbar
N = 0;
fid = fopen([mfilename('fullpath'),'.m'],'r');
while 1;
  tline = fgetl(fid);
  if tline == -1; break; end; % EOF
  
  if strncmp(strtrim(tline),'updatewaitbar',13);
    N = N+1;
  end;
end;
fclose(fid);
updatewaitbar(0); % diplay a status bar to the user

if navdir(end) ~= filesep;
  navdir(end+1) = filesep; % terminate with directory separator
end;
dir_t.syscfg   = dir([navdir,'*.syscfg']);
dir_t.topside  = dir([navdir,'*.TOPSIDE.auv']);
dir_t.raw      = dir([navdir,'*.RAW.auv']);
dir_t.modemdat = dir([navdir,'*.MODEMDAT.auv']);
dir_t.ctl      = dir([navdir,'*.CTL.auv']);

  
%==========================================================
% find the syscfg file: associate files by .syscfg
%==========================================================
Time1 = timestr2struct(filename);
Time2 = timestr2struct('99999999_9999');
foundit = false;
if ~isempty(dir_t.syscfg);
  len = length(dir_t.syscfg);
  for ii=1:len;
    if strcmp(dir_t.syscfg(ii).name,filename);
      if (ii+1)<=len;
        Time2 = timestr2struct(dir_t.syscfg(ii+1).name);
      end;
      foundit = true;
      break;
    end;
  end;
end;
if foundit; 
  syscfg = filename;
else; 
  % exit because we couldn't find syscfg in directory
  error(sprintf('%s is not a valid .syscfg file',[navdir,filename]));
end;

%==========================================================
% remove any old temporary files with the same timestamp
%==========================================================
dir_t.tmp  = dir([tempdir,Time1.str,'*.auv']);
if ~isempty(dir_t.tmp);
  cmd = ['/bin/rm -f ',[tempdir,Time1.str,'*.auv']];
  system(cmd);
end;


%==========================================================
% grep the .auv files associated with filename
%==========================================================
grepLogFiles('TOPSIDE',Time1,Time2,navdir,dir_t);
grepLogFiles('RAW',Time1,Time2,navdir,dir_t);
grepLogFiles('MODEMDAT',Time1,Time2,navdir,dir_t);
grepLogFiles('CTL',Time1,Time2,navdir,dir_t);


%==========================================================
% begin reading in of individual sensor data
%==========================================================
a = [tempdir,Time1.str];

% read the trajectory file produced from CTL
updatewaitbar(N,'REF');
b = strcat(a,'.REF.puv');
if exist(b,'file');
  nav_t.TRAJ = text2struct(b, '%*s %f %f %f %f %f %f %f %f %f %f %f %f %f',...
			  'unixtime',...
			  'depth',...
			  'depth_vel',...
			  'heading',...
			  'heading_vel',...
			  'surge_speed',...
			  'surge_acc',...
			  'depth_goal',...
			  'heading_goal',...
			  'surge_speed_goal',...
			  'sway_speed',...   
			  'sway_acc',...
			  'sway_speed_goal');
end;


% read the estimator file produced from CTL
updatewaitbar(N,'EST');
b = strcat(a,'.EST.puv');
if exist(b,'file');
  nav_t.EST = text2struct(b,'%*s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f',...
			 'unixtime',...
			 'x_lbl',...
			 'y_lbl',...
			 'x_dop',...
			 'y_dop',...
			 'x_dop_lbl',...
			 'y_dop_lbl',...
			 'x_ctl',...
			 'y_ctl',...
			 'vx',...
			 'vy',...
			 'vz',...
			 'depth',...
			 'depth_rate',...
			 'altitude',...
			 'heading',...
			 'heading_rate',...
			 'roll',...
			 'pitch');
end;

  
% read the modes file produced from CTL
updatewaitbar(N,'MDS');
b = strcat(a,'.MDS.puv');
if exist(b,'file');
  nav_t.MDS = text2struct(b, '%*s %f %d %d %d %d %d %d %f',...
			 'unixtime',...
			 'hm',...
			 'tm',...
			 'dm',...
			 'ht',...
			 'tt',...
			 'dt',...
			 'altitude');
end;
  
  
% read goal_setpoint file produced from CTL
updatewaitbar(N,'GSP');
b = strcat(a,'.GSP.puv');
if exist(b,'file');
  nav_t.GOAL = text2struct(b, '%*s %f %d %s %f %f %f %f %f %f %f %f %f',...
			  'unixtime',...
			  'goal_id',...
			  'goal_str',...
			  'xpos1',...
			  'ypos1',...
			  'zpos1',...
			  'xpos2',...
			  'ypos2',...
			  'zpos2',...
			  'heading',...
			  'xy_vel',...
			  'z_vel');
end;
  

% read the thruster file produced from CTL
updatewaitbar(N,'THR');
b = strcat(a,'.THR.puv');
if exist(b,'file');
  nav_t.THR = text2struct(b, '%*s %f %f %f %f %f',...
			 'unixtime',...
			 'port',...
			 'stbd',...
			 'vert',...
			 'lat'); 
end;

  
% read the pixelfly file produced from RAW
updatewaitbar(N,'PIXELFLY');
b = strcat(a,'.PIXELFLY.puv');
if exist(b,'file');
  nav_t.PXF = text2struct(b, '%*s %f %s %dC',...
			  'unixtime',...
			  'imgname',...
			  'camtemp');
  nav_t.PXF.imgnum = str2num(nav_t.PXF.imgname(:,end-7:end-4));
end;
  

% read the IMAGENEX file produced from RAW
updatewaitbar(N,'IMAGENEX');
b = strcat(a,'.IMAGENEX.puv');
if exist(b,'file');
  nav_t.IMA  = text2struct(b, '%*s %f ang:%f ProfRng:%f',...
			  'unixtime',...
			  'ang',...
			  'ProfRng');
end;


% read the RDI file produced from RAW
% three different data strings to handle: 
% 1) older data, 
% 2) newer water track data from pmfs2005
% 3) water track data and u,v,w velocities
updatewaitbar(N,'RDI');
b = strcat(a,'.RDI.puv');
if exist(b,'file')
  % check for water track fields
  fid = fopen(b);
  first_line = fgetl(fid);
  fclose(fid);
  if(~isempty(findstr(first_line,'wvx:')) && ~isempty(findstr(first_line,'u:')));
    disp('PARSING NEW RDI');
    fmt = '%*s %f alt:%f r1:%f r2:%f r3:%f r4:%f h:%f p:%f r:%f vx:%f vy:%f vz:%f u:%f v:%f w:%f nx:%f ny:%f nz:%f COG:%f SOG:%f bt_status:%f h_true:%f p_gimbal:%f wvx:%f wvy:%f wvz:%f wu:%.3f wv:%.3f ww:%.3f wln:%f wlf:%f wt_status:%f btverr:%.3f';
    fields = {'unixtime',...
              'altitude',...
              'r1',...
              'r2',...
              'r3',...
              'r4',...
              'heading_raw',...
              'pitch_raw',...
              'roll',...
              'vx',...
              'vy',...
              'vz',...
              'u',...  
              'v',...
              'w',...
              'nx',...
              'ny',...
              'nz',...
              'cog',...
              'sog',...
              'bt_status',...
              'heading',...
              'pitch',...
              'wvx',... 
              'wvy',...
              'wvz',...
              'wu',...
              'wv',...
              'ww',...  
              'wln',...
              'wlf',...
              'wt_status',...
              'btv_err'};

    % check for strain gage depth
    if findstr(first_line,'depth:');
      fmt = [fmt,' depth:%.3f'];
      fields{end+1} = 'depth';
    end;
    nav_t.RDI = text2struct(b, fmt, fields{:});
  elseif(~isempty(findstr(first_line,'wvx')));
    disp('PARSING WATER-TRACK RDI');
    nav_t.RDI = text2struct(b, ['%*s %f alt:%f r1:%f r2:%f r3:%f r4:%f h:%f p:%f r:%f vx:%f vy:%f vz:%f nx:%f ny:%f nz:%f COG:%f SOG:%f bt_status:%f h_true:%f p_gimbal:%f wvx:%f wvy:%f wvz:%f wln:%f wlf:%f wt_status:%f'],...
			    'unixtime',...
			    'altitude',...
			    'r1',...
			    'r2',...
			    'r3',...
			    'r4',...
			    'heading_raw',...
			    'pitch_raw',...
			    'roll',...
			    'vx',...
			    'vy',...
			    'vz',...
			    'nx',...
			    'ny',...
			    'nz',...
			    'cog',...
			    'sog',...
			    'bt_status',...
			    'heading',...
			    'pitch',...
			    'wvx',... 
			    'wvy',...
			    'wvz',...
			    'wln',...
			    'wlf',...
			    'wt_status');
    if RDIWORLDCFG;
      % do nothing, vx,vy,vz are correct
      disp('CONFIGURED FOR RDI WORLD-FRAME VELOCITIES');
    else;
      % map vx,vy,vz to u,v,w
      disp('CONFIGURED FOR RDI BODY-FRAME VELOCITIES');
      nav_t.RDI.u =  nav_t.RDI.vy;
      nav_t.RDI.v =  nav_t.RDI.vx;
      nav_t.RDI.w = -nav_t.RDI.vz;
      nav_t.RDI   = rmfield(nav_t.RDI,{'vx','vy','vz'});
    end;
  else; % orig format
    disp('PARSING ORIG RDI');
    nav_t.RDI = text2struct(b, ['%*s %f alt:%f r1:%f r2:%f r3:%f r4:%f h:%f p:%f r:%f vx:%f vy:%f vz:%f nx:%f ny:%f nz:%f COG:%f SOG:%f bt_status:%f h_true:%f p_gimbal:%f'],...
			    'unixtime',...
			    'altitude',...
			    'r1',...
			    'r2',...
			    'r3',...
			    'r4',...
			    'heading_raw',...
			    'pitch_raw',...
			    'roll',...
			    'vx',...
			    'vy',...
			    'vz',...
			    'nx',...
			    'ny',...
			    'nz',...
			    'cog',...
			    'sog',...
			    'bt_status',...
			    'heading',...
			    'pitch');
    if RDIWORLDCFG;
      % do nothing, vx,vy,vz are correct
      disp('CONFIGURED FOR RDI WORLD-FRAME VELOCITIES');
    else;
      % map vx,vy,vz to u,v,w
      disp('CONFIGURED FOR RDI BODY-FRAME VELOCITIES');
      nav_t.RDI.u =  nav_t.RDI.vy;
      nav_t.RDI.v =  nav_t.RDI.vx;
      nav_t.RDI.w = -nav_t.RDI.vz;
      nav_t.RDI   = rmfield(nav_t.RDI,{'vx','vy','vz'});
    end;
  end;
end;


% read the PAROSCI file produced from RAW
updatewaitbar(N,'PAROSCI');
b = strcat(a,'.PAROSCI.puv');
if exist(b,'file');
  nav_t.PARO  = text2struct(b, '%*s %f %f',...
			   'unixtime',...
			   'depth');
end;


% read the CROSSBOW file produced from RAW
updatewaitbar(N,'CROSSBOW');
b = strcat(a,'.CROSSBOW.puv');
if exist(b,'file');
  nav_t.XBOW = text2struct(b, '%*s %f h:%f  p:%f  r:%f  hr:%f  pr:%f  rr:%f  T:%f',... 
			   'unixtime',...
			   'heading',...
			   'pitch',...
			   'roll',...
			   'hr',...
			   'pr',...
			   'rr',...
			   'temp');
end;


% read the PHINS heading file produced from RAW
updatewaitbar(N,'PHINS');
b = strcat(a,'.PHINS.puv');
if exist(b,'file');
  nav_t.PHINS = text2struct(b, '%*[^:]: %f H:%f P:%f R:%f',...
			    'unixtime', ...
			    'heading', ...
			    'pitch', ...
			    'roll');
end;


% read the persistor file produced from RAW
updatewaitbar(N,'PERSISTOR');
b = strcat(a,'.PERSISTOR.puv');
if exist(b,'file');
  nav_t.THU = text2struct(b, '%*s %f %f %f %f %f',...
			  'unixtime',...
			  'port',...
			  'stbd',...
			  'vert',...
			  'lat');
end;


% read the THR_VERT file produced from RAW
updatewaitbar(N,'THR_VERT');
b = strcat(a,'.THR_VERT.puv');
if exist(b,'file');
  nav_t.THR_VERT = text2struct(b, '%*s %f RPM:%f A:%f V:%f T:%f',...
			       'unixtime',...
			       'speed',...
			       'current',...
			       'voltage',...
			       'temp');
end;
  

% read the THR_PORT file produced from RAW
updatewaitbar(N,'THR_PORT');
b = strcat(a,'.THR_PORT.puv');
if exist(b,'file');
  nav_t.THR_PORT = text2struct(b, '%*s %f RPM:%f A:%f V:%f T:%f',...
			       'unixtime',...
			       'speed',...
			       'current',...
			       'voltage',...
			       'temp');
end;


% read the THR_STBD file produced from RAW
updatewaitbar(N,'THR_STBD');
b = strcat(a,'.THR_STBD.puv');
if exist(b,'file');
  nav_t.THR_STBD = text2struct(b, '%*s %f RPM:%f A:%f V:%f T:%f',...
			       'unixtime',...
			       'speed',...
			       'current',...
			       'voltage',...
			       'temp');
end;
  

% read the SEABIRD file produced from RAW
updatewaitbar(N,'SEABIRD');
b = strcat(a,'.SEABIRD.puv');
if exist(b,'file');
  nav_t.CTD = text2struct(b, '%*s %f cond:%f temp:%f sal:%f pres:%f sos:%f',...
			  'unixtime',...
			  'cond',...
			  'temp',...
			  'sal',...
			  'pres',...
			  'sos');
end;


% read the METS file produced from RAW
updatewaitbar(N,'METS');
b = strcat(a,'.METS.puv');
if exist(b,'file');
  nav_t.METS = text2struct(b, '%*s %f m:%f t:%f r:%f c:%f',...
			   'unixtime',...
			   'm',...
			   't',...
			   'r',...
			   'c');
end;


% read the FLOUROMETERS file produced from RAW
updatewaitbar(N,'FLR');
b = strcat(a,'.FLR.puv');
if exist(b,'file');
  fid = fopen(b);
  first_line = fgetl(fid);
  fclose(fid);
  if ~isempty(findstr(first_line,'uran:'));
    disp('PARSING NEW FLR');
    nav_t.FLR = text2struct(b, '%*s %f cdom:%f chlor:%f btex:%f uran:%f',...
			    'unixtime',...
			    'cdom',...
			    'chlor',...
			    'btex',...
			    'uran');
  else;
    disp('PARSING OLD FLR');
    nav_t.FLR = text2struct(b, '%*s %f cdom:%f chlor:%f btex:%f',...
			    'unixtime',...
			    'cdom',...
			    'chlor',...
			    'btex');
  end;
end;


% read the OPTODE file produced from RAW
updatewaitbar(N,'OPTODE');
b = strcat(a,'.OPTODE.puv');
if exist(b,'file');
  nav_t.OPTODE = text2struct(b, '%*s %f t:%f s:%f c:%f',...
			     'unixtime',...
			     'temp',...
			     'psat',...
			     'conc');
end;


% read the OCTANS file produced from RAW
updatewaitbar(N,'OCTANS');
b = strcat(a,'.OCTANS.puv');
if exist(b,'file');
  nav_t.OCTANS = text2struct(b,'%*s %f h:%f p:%f r:%f hr:%f pr:%f rr:%f ax:%f ay:%f az:%f heave:%f',... 
			     'unixtime',...
			     'heading',...
			     'pitch',...
			     'roll',...
			     'hr',...
			     'pr',...
			     'rr',...
			     'acc_x',...
			     'acc_y',...
			     'acc_z',...
			     'heave');
end;


% read the CATOA file produced from RAW or NAT
updatewaitbar(N,'CATOA');
b = strcat(a,'.CATOA.puv');
if exist(b,'file');
  try; % .RAW.auv format
    nav_t.CATOA = text2struct(b, '%*s %f $CATOA,%11s,%d%*s',...
                              'unixtime',...
                              'toa_str',...
                              'mode');
  catch; % .NAT.auv format
    nav_t.CATOA = text2struct(b, '%*s %f A $CATOA,%11s,%d%*s',...
                              'unixtime',...
                              'toa_str',...
                              'mode');    
  end;
  % convert TOA string to unixtime assuming that the YYMMDD is correct in unixtime
  [year,month,day,hour,minute,second] = sec_to_ymdhms(nav_t.CATOA.unixtime);
  nav_t.CATOA.toa = unixtime(year,month,day,...
			     str2num(nav_t.CATOA.toa_str(:,1:2)),...
			     str2num(nav_t.CATOA.toa_str(:,3:4)),...
			     str2num(nav_t.CATOA.toa_str(:,5:end)));

  % OWTT hack for now until timestamp gets encoded in packet.
  % this assumes there aren't any modulo 1 second problems.
  nav_t.CATOA.tos = floor(nav_t.CATOA.toa); % time-of-send
  nav_t.CATOA.owtt = nav_t.CATOA.toa - floor(nav_t.CATOA.toa);
  
  nav_t.CATOA = reorderstructure(nav_t.CATOA,'rovtime','unixtime','tos','toa','toa_str','mode','owtt');
end;


% read the PPS file produced from RAW
updatewaitbar(N,'PPS');
b = strcat(a,'.PPS.puv');
if exist(b,'file');
    nav_t.PPS = text2struct(b,'%*s %f clock_timestamp:%f clock_valid=%d sync_timestamp:%d sync_counts:%d sync_state:%d gps:%d pps:%d offset_counts:%d offset:%f T:%f',...
                            'unixtime',...
                            'clktime',...
                            'clkvalid',...
                            'sync_time',...
                            'sync_counts',...
                            'sync_state',...
                            'gps_state',...
                            'pps_state',...
                            'offset_counts',...
                            'offset',...
                            'T');
%   nav_t.PPS = text2struct(b,'%*s %f gtimestamp%*c%d sync_rov_timestamp:%f sync_gps_timestamp:%d sync_counts:%d src:%d gps:%d pps:%d offset_counts:%d offset:%f T:%f',...
% 			  'unixtime',...
% 			  'grptime',...
% 			  'sync_unixtime',...
% 			  'sync_gpstime',...
% 			  'sync_counts',...
% 			  'sync_src',...
% 			  'gps_state',...
% 			  'pps_state',...
% 			  'offset_counts',...
% 			  'offset',...
% 			  'T');
end;


% read the GPS file produced from RAW
updatewaitbar(N,'GPS');
b = strcat(a,'.GPS.puv');
if exist(b,'file');
  nav_t.GPS = text2struct(b,'%*s %f gps_timestamp:%f lat:%f lon:%f altitude:%f hdop:%f pdop:%f vdop:%f pdop10:%d tdop10:%d hpe:%f vpe:%f epe:%f sog:%f cog_true:%f mag_var:%f mag_dir:%c nsat:%d fix_type:%d fix_quality:%d fix_src:%c status:%c geoid:%f',...
			  'unixtime',...
			  'gpstime',...
			  'lat',...
			  'lon',...
			  'alt',...
			  'hdop',...
			  'pdop',...
			  'vdop',...
			  'pdop10',...
			  'tdop10',...
			  'hpe',...
			  'vpe',...
			  'epe',...
			  'sog',...
			  'cog',...
			  'magvar',...
			  'magdir',...
			  'nsat',...
			  'fixtype',...
			  'fixqual',...
			  'fixsrc',...
			  'status',...
              'geoid');
end;


% read the LBL file produced from MODEMDAT
updatewaitbar(N,'LBL');
b = strcat(a,'.LBL.puv');
if exist(b,'file');
  try;
    nav_t.LBL = text2struct(b, '%*s %f %f %f %f %f', ... 
                            'unixtime',...
                            'owtt1',...
                            'owtt2',...
                            'owtt3',...
                            'owtt4');
  catch; % old LBL format
    nav_t.LBL = text2struct(b, '%f: %*s %f %f %f %f', ... 
                            'unixtime',...
                            'owtt1',...
                            'owtt2',...
                            'owtt3',...
                            'owtt4');    
  end;
  % reorganize OWTT into a M x 4 array
  nav_t.LBL.owtt = [nav_t.LBL.owtt1,nav_t.LBL.owtt2,nav_t.LBL.owtt3,nav_t.LBL.owtt4];
  nav_t.LBL = rmfield(nav_t.LBL,{'owtt1','owtt2','owtt3','owtt4'});
  
  % parse out beacon positions from .syscfg
  nav_t.LBL.lbl_xponder1 = zeros(1,6);
  nav_t.LBL.lbl_xponder2 = zeros(1,6);
  nav_t.LBL.lbl_xponder3 = zeros(1,6);
  nav_t.LBL.lbl_xponder4 = zeros(1,6);
  nav_t.LBL.lbl_baseline = '';
  nav_t.LBL.lbl_sos = 0;
  fid = fopen([navdir,syscfg]);
  while (fid ~= -1);
    fline = lower(fgetl(fid));
    if fline == -1; break; end; % EOF
    
    if strncmp(fline,'lbl_xponder1:',13);
      nav_t.LBL.lbl_xponder1 = sscanf(fline,'lbl_xponder1: %f %f %f %f %f %f',6)';
    elseif strncmp(fline,'lbl_xponder2:',13);
      nav_t.LBL.lbl_xponder2 = sscanf(fline,'lbl_xponder2: %f %f %f %f %f %f',6)';
    elseif strncmp(fline,'lbl_xponder3:',13);
      nav_t.LBL.lbl_xponder3 = sscanf(fline,'lbl_xponder3: %f %f %f %f %f %f',6)';
    elseif strncmp(fline,'lbl_xponder4:',13);
      nav_t.LBL.lbl_xponder4 = sscanf(fline,'lbl_xponder4: %f %f %f %f %f %f',6)';
    elseif strncmp(fline,'lbl_baseline:',13);
      nav_t.LBL.lbl_baseline = sscanf(fline,'lbl_baseline: %s',1);
    elseif strncmp(fline,'lbl_sos:',8);
      nav_t.LBL.lbl_sos = sscanf(fline,'lbl_sos: %f',1);
    end;
  
  end; % while 1
  fclose(fid);
  
  % parse out turn-around-time from .MICROMODEM.puv
  fname = [a,'.MICROMODEM.puv'];
  fid = fopen(fname);
  while (fid ~= -1);
    fline = fgetl(fid);
    if fline == -1; break; end; % EOF
    
    if findstr(fline,'$CACFG,TAT,');
      nav_t.LBL.lbl_tat = sscanf(fline,'%*s %*s $CACFG,TAT,%d%*s') * 1e-3; % convert from [ms] to [s]
      %nav_t.LBL.ttwtt = nav_t.LBL.owtt*2+nav_t.LBL.lbl_tat;
      break;
    end;
  end; % while 1
  
  %nav_t.LBL = reorderstructure(nav_t.LBL,'twtt','ttwtt');
  
end;


% read the GYRO file produced from TOPSIDE
updatewaitbar(N,'GYRO');
b = strcat(a,'.GYRO.puv');
if exist(b,'file');
  nav_t.GYRO = text2struct(b, '%*s %f %f',...
			   'unixtime',...
			   'heading');
end;


% read the LXT file produced from TOPSIDE
updatewaitbar(N,'LXT');
b = strcat(a,'.LXT.puv');
if exist(b,'file');
  nav_t.LXT = text2struct(b, '%*s %f rng:%f brg:%f azm:%f',...
			  'unixtime',...
			  'rng',...
			  'brg',...
			  'azm');
end;


% recursively loop through data structure and set
% NO_VALUE sentinals to NaN
updatewaitbar(N,'NO_VALUE');
sensors = fieldnames(nav_t);
STARTTIME = inf;
ENDTIME = -inf;
for ii=1:length(sensors);
  sensor = sensors{ii};
  if isfield(nav_t.(sensor),'unixtime');
    STARTTIME = min(STARTTIME,nav_t.(sensor).unixtime(1));
    ENDTIME = max(ENDTIME,nav_t.(sensor).unixtime(end));
  end;
  nav_t.(sensor) = fix_no_value(nav_t.(sensor),NO_VALUE);
end;
% reorganize the nav_t struct alphabetically by sensor
sensors = sort(sensors);
nav_t = reorderstructure(nav_t,sensors{:});

% convert mission time from unix seconds into
% "seconds into mission"
for ii=1:length(sensors);
  sensor = sensors{ii};
  nav_t.(sensor).rovtime = nav_t.(sensor).unixtime - STARTTIME;
  nav_t.(sensor) = reorderstructure(nav_t.(sensor),'rovtime','unixtime');
end;
nav_t.STARTTIME = STARTTIME;
nav_t.ENDTIME   = ENDTIME;


updatewaitbar(-1);
return;


% PRIVATE FUNCTIONS
%===============================================================================
function s_t = fix_no_value(s_t,NO_VALUE);

TOL = 0.1;
fields = fieldnames(s_t);
for ii=1:length(fields);
  field = fields{ii};
  if isstruct(s_t.(field));
    s_t.(field) = fix_no_value(s_t.(field),NO_VALUE);
  elseif isnumeric(s_t.(field));
    kk = find( (s_t.(field)(:) == NO_VALUE) | ...
	       (abs(s_t.(field)(:) - RTOD*NO_VALUE) < TOL) | ...
	       (abs(s_t.(field)(:) - DTOR*NO_VALUE) < TOL) );
    s_t.(field)(kk) = NaN;
  end;
end;


%===================================================================
function time_t = timestr2struct(timestr);

time_t.str      = timestr(1:13);          % YYYYMMDD_HHMM
time_t.year     = str2num(timestr(1:4));
time_t.month    = str2num(timestr(5:6));
time_t.day      = str2num(timestr(7:8));
time_t.hour     = str2num(timestr(10:11));
time_t.min      = str2num(timestr(12:13));
time_t.sec      = 0;
time_t.unixtime = unixtime(time_t.year,time_t.month,time_t.day, ...
			   time_t.hour,time_t.min,time_t.sec);


%====================================================================
function varargout = updatewaitbar(N,sensor);

persistent n handle;

if N == 0;    % initialize the waitbar
  n = 0;
  handle = waitbar(0,'Processing ASCII files ...');
elseif N < 0; % close the waitbar
  close(handle);
else;         % update the waitbar
  n = n+1;
  sensor = strrep(sensor,'_','\_');
  waitbar(n/N,handle,sprintf('Loading %s into MATLAB...',upper(sensor)));
end;
drawnow;


%====================================================================
function grepLogFiles(ftype,Time1,Time2,navdir,dir_t);

switch upper(ftype);
case 'TOPSIDE';
 ext = '.TOPSIDE.auv';
case 'RAW';
 ext = '.RAW.auv'; 
case 'MODEMDAT';
 ext = '.MODEMDAT.auv';
case 'CTL';
 ext = '.CTL.auv'; 
otherwise;
 error(sprintf('Uknown ftype: [%s]',ftype));
end
field = lower(ftype);
process_AUV = which('process_AUV.sh');


files = '';
ftype = lower(ftype);
if ~isempty(dir_t.(field));
  len = length(dir_t.(field));
  for ii=1:len
    Time = timestr2struct(dir_t.(field)(ii).name);
    if (Time.unixtime >= Time1.unixtime && Time.unixtime <  Time2.unixtime)
	files = [files,navdir,dir_t.(field)(ii).name,' '];
    end;
  end;
  system(['/bin/cat ',files,' > ',[tempdir,Time1.str,ext]]);
  system([process_AUV,' -C ',tempdir,' ',[tempdir,Time1.str,ext]]);
end;


%====================================================================
function x = RTOD()
x = 180/pi;

%====================================================================
function x = DTOR()
x = pi/180;
