%FEB 26 2004  JHU tank grid data set BW camera

% issue a "clear all" to remove any persistent function variables residing in memory
clear all;

% temporary defines
OFF = false;
ON  = true;

%======================================
% SETUP DATA PATHS
%======================================
% sysconfig file (full pathname)
TheConfig.Data.navDir = '/files1/data/jhu04/20040228/jhu04-9_gridrot/';
TheConfig.Data.navFile = 'nav_t.mat';
TheConfig.Data.navLoader = [];

% image directory
TheConfig.Data.imageDir  = '/files1/data/jhu04/20040226/jhu04-9_gridrot/images_pxf/';

% image extentions [e.g. tif jpg png etc]
TheConfig.Data.imageExtension = 'png';

% camera calibration file
TheConfig.Data.imageCalibrationFile = '/files1/data/jhu04/calib_jhu20040226_bw.m';

% image sequence to process
% (image #'s starts at zero, add 1 to get correct matlab index)
TheConfig.Data.imageSequence = [25:1:1431]'+1;
TheConfig.Data.imgest = TheConfig.Data.imageSequence-1;

% number of images in sequence
TheConfig.Data.nImages = length(TheConfig.Data.imgest);

% output data directory
TheConfig.Data.outdir = '/files1/processed/van/output/jhu04-9_gridrot/';

% user interaction
TheConfig.Data.vpause = 3;

%=======================================
% SETUP EKF
%=======================================
% use delayed state processing to incorporate rdi displacement
% measurement after serial traffic dropout?
TheConfig.ekf.rdi_delayed_state_processing = OFF;

% use augmented state vector processing to incorporate image
% based pose measurements and smoothing?
TheConfig.ekf.aug_state_filtering = OFF;

% use camera measurements
TheConfig.ekf.use_camera_meas = OFF;

% vehicle process model function handle
TheConfig.ekf.pmfhandle = @pm_constant_velocity;

% standard nav sensors measurement config
TheConfig.ekf.use_rdi   = 2; % 0 - off
                            % 1 - u,v,w,r,p,h
                            % 2 - u,v,w
			    % 3 - u,v,w,r,p
TheConfig.ekf.use_xbow  = 0; % 0 - off
                            % 1 - rdot,pdot,hdot
TheConfig.ekf.use_phins = 1; % 0 - off
                            % 1 - r,p,h
			    % 2 - h
TheConfig.ekf.use_paro  = 1; % 0 - off
                            % 1 - z

% state process noise is based upon a vehicle state model constant
% velocity process model with a state vector which looks like:
% Xv = [x_ll, u, y_ll, v, z_ll, w, r, rr, p, pr, h, hr]'
% this form results in more efficient partitioning and better numerical
% conditioning
TheConfig.ekf.Q = diag([[0 0.01 0 0.01 0 0.01],...
		       [0 1.5 0 1.5 0 1.5]*DTOR]).^2;

% define sensor measurement std
TheConfig.nav_t.RDI.sigma.altitude = 0.2 ;       % [m]
TheConfig.nav_t.RDI.sigma.heading  = 2.0 * DTOR; % [rad] % 1.0
TheConfig.nav_t.RDI.sigma.pitch    = 0.5 * DTOR; % [rad] % 0.5
TheConfig.nav_t.RDI.sigma.roll     = 0.5 * DTOR; % [rad] % 0.5
TheConfig.nav_t.RDI.sigma.u        = 0.005;      % [m/s] % 0.01
TheConfig.nav_t.RDI.sigma.v        = 0.005;      % [m/s]
TheConfig.nav_t.RDI.sigma.w        = 0.01;      % [m/s]
TheConfig.nav_t.XBOW.sigma.hr      = 1.0 * DTOR; % [rad/s] % 1.0
TheConfig.nav_t.XBOW.sigma.pr      = 1.0 * DTOR; % [rad/s]
TheConfig.nav_t.XBOW.sigma.rr      = 1.0 * DTOR; % [rad/s]
TheConfig.nav_t.PARO.sigma.depth   = 0.01;       % [m]   % 0.05
TheConfig.nav_t.PHINS.sigma.heading= 0.5 * DTOR; % [rad]
TheConfig.nav_t.PHINS.sigma.pitch  = 0.25 * DTOR; % [rad]
TheConfig.nav_t.PHINS.sigma.roll   = 0.25 * DTOR; % [rad]
TheConfig.nav_t.KVH.sigma.heading  = 2.0 * DTOR; % [rad]
TheConfig.nav_t.KVH.sigma.pitch    = 1.0 * DTOR; % [rad]
TheConfig.nav_t.KVH.sigma.roll     = 1.0 * DTOR; % [rad]
TheConfig.nav_t.LBL.sigma.X        = 0.01;       % [m]
TheConfig.nav_t.LBL.sigma.Y        = 0.01;       % [m]


%=======================================
% STATIC RIGID BODY TRANSFORMS
%=======================================
%======================================
% DOPPLER TO VEHCLE
%--------------------------------------
% calibrated rotation matrix provied by James Kinsey 03/08/2004
TheConfig.SensorXform.RDI.Rvs = ...
    [ 0.704912855291639, -0.707889187915883,  0.0446179792951256; ...
      0.70792544187841,   0.706065131254346,  0.0177087314068754; ...
     -0.04403901890195,   0.0191030902886104, 0.998847153850667];
%TheConfig.SensorXform.RDI.Rvd = rotxyz([0,0,45]*DTOR); % eyeball approximate
TheConfig.SensorXform.RDI.tvs_v = [0.08, 0, 0.294]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.RDI.Tvs = [TheConfig.SensorXform.RDI.Rvs, TheConfig.SensorXform.RDI.tvs_v; ...
		          0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.RDI.x_vs = [TheConfig.SensorXform.RDI.tvs_v; ...
            		   rot2rph(TheConfig.SensorXform.RDI.Rvs)];
%======================================
% PHINS TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.PHINS.Rvs = rotxyz([0,0,90]*DTOR);
TheConfig.SensorXform.PHINS.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.PHINS.Tvs = [TheConfig.SensorXform.PHINS.Rvs, TheConfig.SensorXform.PHINS.tvs_v; ...
		            0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.PHINS.x_vs = [TheConfig.SensorXform.PHINS.tvs_v; ...
		             rot2rph(TheConfig.SensorXform.PHINS.Rvs)];

%======================================
% XBOW TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.XBOW.Rvs = rotxyz([-90,0,-90]*DTOR);
TheConfig.SensorXform.XBOW.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.XBOW.Tvs = [TheConfig.SensorXform.XBOW.Rvs, TheConfig.SensorXform.XBOW.tvs_v; ...
  		           0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.XBOW.x_vs = [TheConfig.SensorXform.XBOW.tvs_v; ...
		            rot2rph(TheConfig.SensorXform.XBOW.Rvs)];

%======================================
% PARO TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.PARO.Rvs = rotxyz([0,0,0]*DTOR);
TheConfig.SensorXform.PARO.tvs_v = [-0.017, -0.3500, 0.2250]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.PARO.Tvs = [TheConfig.SensorXform.PARO.Rvs, TheConfig.SensorXform.PARO.tvs_v; ...
  		           0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.PARO.x_vs = [TheConfig.SensorXform.PARO.tvs_v; ...
		            rot2rph(TheConfig.SensorXform.PARO.Rvs)];

%=========================================
% CAMERA FRAME TO VEHICLE FRAME
%=========================================
% ^ Xv         x---> Xc
% |            |
% x---> Yv     V Yc
%======================================
% normally the rotation matrix is:
TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 90]*DTOR);
% but if the camera was mounted "backwards" on the
% vehicle (i.e. rotated 180 from what it
% should have been) the rotation matrix is instead:
%TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 -90]*DTOR);
% vector from vehicle origin to camera origin expressed 
% in vehicle frame
TheConfig.SensorXform.PXF.tvs_v = [0.045, -0.17, 0.3935]'; % meters PXF camera
%TheConfig.SensorXform.tvc_v = [0.015, 0.15, 0.3585]'; % meters JHU camera
% 4x4 coordinate transformation from camera to vehicle
TheConfig.SensorXform.PXF.Tvs = [TheConfig.SensorXform.PXF.Rvs, TheConfig.SensorXform.PXF.tvs_v; ...
		          0  0  0  1];
% 6x1 pose vector
TheConfig.SensorXform.PXF.x_vs = [TheConfig.SensorXform.PXF.tvs_v; ...
		           rot2rph(TheConfig.SensorXform.PXF.Rvs)];

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
% 6x1 pose vector
TheConfig.SensorXform.x_wl = [TheConfig.SensorXform.twl_w; ...
		       rot2rph(TheConfig.SensorXform.Rwl)];


%=======================================
% IMAGE SPECIFIC
%=======================================
% create a mask defining valid region of image to work with
TheConfig.image.mask = load('radmask'); 
% threshold for proposing spatial neighbors
TheConfig.image.rho = 1.0; % [m]

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
% normalized correlation threshold is related to normalized SSD via:
% thresh_SSD = 2(1-thresh_NC)
% thresh_NC  = 1-thresh_SSD/2
TheConfig.ImageFeature.simthresh = 0.75;

