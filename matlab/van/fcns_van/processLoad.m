%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    20040227        rme         Created from loaddata_seabed.m
%    04-01-2004      rme         Updated TheConfig.nav_t to TheConfig.SensorNoise
%    04-08-2004      rme         Updated to use mindex_t structure and
%                                removed nav_t.sigma field
%    09-02-2004      rme         Added TheConfig.Heuristic to control
%                                additional sensor processing
%    11-05-2004      rme         Updated fprintf print commands
%    01-04-2006      rme         Added passing of TheConfig.Data.navLoaderOptArgs to feval
%    02-03-2006      rme         Renamed from loaddata.m to processLoad.m
%    02-07-2007      rme         Made TheConfig.Data.navLoaderArgs contain all navLoader args.

%====================================
% LOAD VEHICLE NAVIGATION DATA
%====================================
TheConfig.Data.navMatFile = strcat(TheConfig.Data.outdir,'nav_t.mat');
if exist(TheConfig.Data.navMatFile,'file')
  load(TheConfig.Data.navMatFile);
else
  % load navigation data into nav data structure
  nav_t = feval(TheConfig.Data.navLoader, TheConfig.Data.navLoaderArgs{:});

  % convert from degrees to radians and cook raw sensor
  % measurements to represent vehicle frame
  % also look for obvious outliers and toss 'em
  nav_t = cook_nav(nav_t,TheConfig);
  
  %======================================
  % FIND NAVIGATION DATA TIMES CLOSEST 
  % TO IMAGE TIMES
  %======================================
  % camind is index into navigation corresponding
  % to camera image
  if isfield(nav_t,'PXF');
    nav_t.PXF.camind = find_nav_cam_index(nav_t,'PXF');
    %nav_t.DSPL.camind = find_nav_cam_index(nav_t,'DSPL');
  end;

  %======================================
  % SAVE NAV DATA STRUCTURE
  %======================================
  if ~exist(TheConfig.Data.outdir,'dir')
    mkdir(TheConfig.Data.outdir);
  end
  save(TheConfig.Data.navMatFile,'nav_t');
end
%====================================
% END LOADING VEHICLE NAVIGATION DATA
%====================================

% use heading bias correction calculated by oscar
if exist(TheConfig.Data.headingBiasFile,'file')
  fprintf('==>%s: Compass correction file exists. Correcting heading... ',mfilename);
  tmp = load(TheConfig.Data.headingBiasFile); % loads P
  P = tmp.P;
  P(end) = 0; % remove DC offset
  nav_t.RDI.heading = compass_correction(nav_t.RDI.heading,P);
  nav_t.RDI.heading_cooked = compass_correction(nav_t.RDI.heading_cooked,P);
  fprintf('done\n');
end
  
if TheConfig.Heuristic.RDI_fix_srange
  % correct RDI ranges for cos(30) factor so they represent true
  % slant ranges
  fprintf('==>%s: Correcting DVL slant ranges by 1/cos(30)... ',mfilename);
  nav_t.RDI.r1 = nav_t.RDI.r1/cos(30*DTOR);
  nav_t.RDI.r2 = nav_t.RDI.r2/cos(30*DTOR);
  nav_t.RDI.r3 = nav_t.RDI.r3/cos(30*DTOR);
  nav_t.RDI.r4 = nav_t.RDI.r4/cos(30*DTOR);
  fprintf('done\n');
end

if TheConfig.Heuristic.XBOW_fix_timebase
  % offset XBOW time
  delta = 0.2;
  fprintf('==>%s: Correcting XBOW rovtime by delta=%.2f... ',mfilename,delta);
  nav_t.XBOW.rovtime = nav_t.XBOW.rovtime-delta;
  fprintf('done\n');
end

if TheConfig.Heuristic.XBOW_fake_from_phins
  fprintf('==>%s: Faking XBOW measurements using PHINS time derivative... ',mfilename);
  t = nav_t.PHINS.rovtime(2:end);
  dt = diff(nav_t.PHINS.rovtime);
  rdot = diff(nav_t.PHINS.roll_cooked)./dt;
  pdot = diff(nav_t.PHINS.pitch_cooked)./dt;
  hdot = diff(nav_t.PHINS.heading_cooked)./dt;
  rph = [nav_t.PHINS.roll_cooked, nav_t.PHINS.pitch_cooked, nav_t.PHINS.heading_cooked]';
  % map to body-frame pqr
  pqr = zeros(3,length(dt));
  for k=1:length(dt)
    rph_dot = [rdot(k); ...
	       pdot(k); ...
	       hdot(k);];
    pqr(:,k) = euler2body(rph_dot,rph(:,k+1));
  end
  nav_t.XBOW.rovtime = t;
  nav_t.XBOW.rr = pqr(1,:)';
  nav_t.XBOW.pr = pqr(2,:)';
  nav_t.XBOW.hr = pqr(3,:)';
  fprintf('done\n');
end
