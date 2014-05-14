%2002 BERMUDA DATA SET

% issue a "clear all" to remove any persistent function variables residing in memory
clear all;

% temporary defines
OFF = false;
ON  = true;

%======================================
% SETUP DATA PATHS
%======================================
% sysconfig file (full pathname)
TheConfig.Data.navDir = '/files1/data/bermuda02/Log/d20020827/';
TheConfig.Data.navFile = '20020827_2149.SEABED.syscfg';
TheConfig.Data.navLoader = @loadseabed;

% image directory
TheConfig.Data.imageDir  = ['/files1/data/bermuda02/Images/i20020827/' ...
		         '20020827_2149.navsurv/GrayProcessed_8/'];

% image extentions [e.g. tif jpg png etc]
TheConfig.Data.imageExtension = 'tif';

% camera calibration file
TheConfig.Data.imageCalibrationFile = '/files1/data/bermuda02/Color_Calib_Results_20021125.m';

% image sequence to process
% (image #'s starts at zero, add 1 to get correct matlab index)
TheConfig.Data.imageSequence = [0:1:572]'+1;
TheConfig.Data.imgest = TheConfig.Data.imageSequence;

% number of images in sequence
TheConfig.Data.nImages = length(TheConfig.Data.imgest);

% output data directory
TheConfig.Data.outdir = '/files1/processed/van/output/20020827_2149/';

% user interaction
TheConfig.Data.vpause = 0;

%=======================================
% SETUP EKF
%=======================================
% use delayed state processing to incorporate rdi displacement
% measurement after serial traffic dropout?
TheConfig.ekf.rdi_delayed_state_processing = ON;

% use augmented state vector processing to incorporate image
% based pose measurements and smoothing?
TheConfig.ekf.aug_state_filtering = OFF;

% use camera measurements
TheConfig.ekf.use_camera_meas = OFF;

% vehicle process model function handle
%TheConfig.ekf.pmfhandle = @pm_constant_velocity;
TheConfig.ekf.pmfhandle = @pm_constant_velocity_hbias;

% state process noise is based upon a vehicle state model constant
% velocity process model with a state vector which looks like:
% Xv = [x_ll, u, y_ll, v, z_ll, w, r, rr, p, pr, h, hr]'
% this form results in more efficient partitioning and better numerical conditioning
TheConfig.ekf.beta_bh = 1/6; % must match value in pm_constant_velocity_hbias.m
TheConfig.ekf.sigma_bh = 0*DTOR;
TheConfig.ekf.Q = diag([[0 0.1 0 0.1 0 0.1] [0 2 0 2 0 2]*DTOR, 0].^2);

% sensor measurement std
TheConfig.nav_t.RDI.sigma.altitude = 0.2;        % [m]
TheConfig.nav_t.RDI.sigma.heading  = 2.0 * DTOR; % [rad] % 1.0
TheConfig.nav_t.RDI.sigma.pitch    = 1.0 * DTOR; % [rad] % 0.5
TheConfig.nav_t.RDI.sigma.roll     = 1.0 * DTOR; % [rad] % 0.5
TheConfig.nav_t.RDI.sigma.u        = 0.02;      % [m/s] % 0.01
TheConfig.nav_t.RDI.sigma.v        = 0.02;      % [m/s]
TheConfig.nav_t.RDI.sigma.w        = 0.02;      % [m/s]
TheConfig.nav_t.XBOW.sigma.hr      = 1.0 * DTOR; % [rad/s] % 1.0
TheConfig.nav_t.XBOW.sigma.pr      = 1.0 * DTOR; % [rad/s]
TheConfig.nav_t.XBOW.sigma.rr      = 1.0 * DTOR; % [rad/s]
TheConfig.nav_t.PARO.sigma.depth   = 0.03;       % [m]   % 0.05

%=======================================
% STATIC RIGID BODY TRANSFORMS
%=======================================
%======================================
% VEHICLE FRAME IS DEFINED COINCIDENT
% WITH DOPPLER FRAME, BUT ORIENTED S.T.
% Xv IS FORWARD, Yv IS STARBOARD, Zv IS DOWN
% AS OPPOSED TO:
% Yd IS FORWARD, Xd IS STARBOARD, Zd IS UP
% VEHICLE FRAME IS WHAT I'M ASSOCIATING
% RPH ATTITUDE MEASUREMENTS TO
%--------------------------------------
% ^ Yd         ^ Xv
% |            |
% o---> Xd     x---> Yv
%======================================
TheConfig.SensorXform.Rvd = [0  1  0; ...
		      1  0  0; ...
		      0  0 -1];
TheConfig.SensorXform.tvd_v = [0 0 0]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.Tvd = [TheConfig.SensorXform.Rvd, TheConfig.SensorXform.tvd_v; ...
  		      0 0 0 1];

%======================================
% WORLD COORDINATE SYSTEM
% Yw NORTH, Xw EAST, Zw UP
% LOCAL LEVEL COORDINATE SYSTEM
% Xl NORTH, Yl EAST, Zl DOWN
%--------------------------------------
% ^ Yworld  North    ^ Xll North     
% |                  |
% o---> Xworld East  x---> Yll  East  
%======================================
TheConfig.SensorXform.Rwl = [0  1  0; ...
		      1  0  0; ...
		      0  0 -1];
TheConfig.SensorXform.twl_w = [0 0 0]';
% 4x4 coordinate transformation from local-level to world
TheConfig.SensorXform.Twl = [TheConfig.SensorXform.Rwl, TheConfig.SensorXform.twl_w; ...
		      0  0  0  1];

%=========================================
% CAMERA FRAME TO VEHICLE FRAME
%=========================================
% ^ Xv         x---> Xc
% |            |
% x---> Yv     V Yc
%======================================
% normally the rotation matrix is:
%TheConfig.SensorXform.Rvc = [0 -1  0; ...
%		      1  0  0; ...
%		      0  0  1]; 
% but if the camera was mounted "backwards" on the
% vehicle (i.e. rotated 180 from what it
% should have been) the rotation matrix is instead:
TheConfig.SensorXform.Rvc = [ 0  1  0; ...
		      -1  0  0; ...
		       0  0  1];
% vector from vehicle origin to camera origin expressed 
% in vehicle frame
TheConfig.SensorXform.tvc_v = [1.40 0 0]'; % meters
% 4x4 coordinate transformation from camera to vehicle
TheConfig.SensorXform.Tvc = [TheConfig.SensorXform.Rvc, TheConfig.SensorXform.tvc_v; ...
		      0  0  0  1];



%=======================================
% IMAGE SPECIFIC
%=======================================
% create a mask defining valid region of image to work with
TheConfig.image.mask = load('radmask'); 


%======================================
% FEATURE VECTOR SPECIFIC
%======================================
% size of feature window [(2w+1)x(2w+1)]
TheConfig.ImageFeature.w = 15;
% number of image feature regions
TheConfig.ImageFeature.tile = [8 10];
% number of feature points to extract
TheConfig.ImageFeature.N = 2000;
% feature vector similarity threshold (normalized correlation score)
% feature vector similarity threshold (normalized correlation score)
% normalized correlation threshold is related to normalized SSD via:
% thresh_SSD = 2(1-thresh_NC)
% thresh_NC  = 1-thresh_SSD/2
TheConfig.ImageFeature.simthresh = 0.65;
