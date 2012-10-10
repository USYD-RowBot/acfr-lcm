%FEB 26 2004  JHU tank grid data set BW camera

% issue a "clear all" to remove any persistent function variables residing in memory
clear all;

% default terminal characteristics
setTerminal('dim');

%======================================
% SETUP DATA PATHS
%======================================
% rootdir
TheConfig.Data.rootdir = '/files1/thesis/van/';

% sysconfig file (full pathname)
TheConfig.Data.navDir = '/files1/data/jhu04/20040226/jhu04-6_gridsurvey3/';
TheConfig.Data.navFile = 'nav_t.mat';
TheConfig.Data.navLoader = @loadjhu;

% image directory
TheConfig.Data.imageDir  = '/files1/data/jhu04/20040226/jhu04-6_gridsurvey3/images_pxf/Fixed/';

% image extentions [e.g. tif jpg png etc]
TheConfig.Data.imageExtension = 'png';

% camera calibration file
TheConfig.Data.imageCalibrationFile = '/files1/data/jhu04/calibration/calib_jhu20040226_bw.m';
TheConfig.Calib = initializeCalib(TheConfig.Data.imageCalibrationFile);

% rdi compass bias file
% specify an empty matrix if no file exists
TheConfig.Data.headingBiasFile = [];

% image sequence to process
% note for this data set, valid images seq is between [50:3240,3242:3410]
TheConfig.Data.imageSequence = [2000:10:3000]; % 608,620 problem
%TheConfig.Data.imageSequence = [1760,2150]; % 608,620 problem
%TheConfig.Data.imageSequence = [2810:10:2900]; % 608,620 problem
%TheConfig.Data.imageSequence = [3000:10:3100]; % 608,620 problem
%TheConfig.Data.imageSequence = [2100:10:2130,2290:10:2320]; % 608,620 problem

% number of images in sequence
TheConfig.Data.nImages = length(TheConfig.Data.imageSequence);

% output data directory
TheConfig.Data.outdir = '/files1/processed/van/output/jhu04-6_gridsurvey3/';

% memory stack size for Image Features
TheConfig.Data.stackSize = 10;

% max survey altitude
TheConfig.Data.maxAltitude = 3; %[m]

% user interaction
TheConfig.Data.vpause = 0;

%=======================================
% SLAM OPTIONS
%=======================================
% use augmented state vector processing to incorporate image
% based pose measurements and smoothing?
TheConfig.Estimator.useDelayedStates = on;

TheConfig.Estimator.useEkf       = off;
TheConfig.Estimator.useEif       = on;
TheConfig.Estimator.inferenceEif = on;

TheConfig.Estimator.useFullStateRecovery = on;
TheConfig.Estimator.useSeifsDA           = off;
TheConfig.Estimator.groupSize            = 3;
TheConfig.Estimator.pcgMaxIter           = 25;
TheConfig.Estimator.estimateCameraXform  = off;
TheConfig.Estimator.record_stats         = on;
TheConfig.Estimator.record_innov         = off;
TheConfig.Estimator.record_ssv           = on;
TheConfig.Estimator.ssvSampleHertz       = 2;
TheConfig.Estimator.ssvSamplePeriod      = 1/TheConfig.Estimator.ssvSampleHertz;
TheConfig.Estimator.max_link_hypoth      = 5;
TheConfig.Estimator.useSavedCameraMeas   = 0; %0 - don't use
                                              %1 - use if available
					      %2 - use if available and skip pairs w/o saved measurement


%=======================================
% CHOOSE SENSOR OBSERVATION MODEL
%=======================================
TheConfig.ObservationModel.RDI   = @om_rdi_uvw;
TheConfig.ObservationModel.XBOW  = @om_xbow_abc;
TheConfig.ObservationModel.PHINS = @om_phins_rph;
TheConfig.ObservationModel.PARO  = @om_paro_z;
TheConfig.ObservationModel.PXF   = [];%@om_camera_Rae21;

%=======================================
% CHOOSE PROCESS MODEL
%=======================================
% vehicle process model function handle
TheConfig.ProcessModel.pmfhandle = @pm_constant_velocity;
%TheConfig.ProcessModel.pmfhandle = @pm_constant_acceleration;

% state process noise terms.  the process model Q matrix is assembled
% based upon TheConfig.ProcessModel.pmfhandle and relevant terms from below.
% note that the terms below represent the process noise simga,
% *not* covariance.
%-----------------------------------------------------------------
% constant velocity sigma terms
TheConfig.ProcessModel.Noise.uvw = 0.01 * [1,1,1/8];       % m/s 0.01
TheConfig.ProcessModel.Noise.abc = 0.160 * DTOR * [1,1,1];  % rad/s 0.10
% constant acceleration sigma terms
TheConfig.ProcessModel.Noise.uvw_dot = [0.006, 0.003, 0.001];  % m/s^2
TheConfig.ProcessModel.Noise.abc_dot = DTOR * [0.2, 0.2, 0.2]; % rad/s^2

%=======================================
% DEFINE NAV SENSOR STATISICS
%=======================================
TheConfig.SensorNoise.RDI.altitude = 0.1;        % [m]
TheConfig.SensorNoise.RDI.heading  = 2.0 * DTOR; % [rad] % 1.0
TheConfig.SensorNoise.RDI.pitch    = 0.5 * DTOR; % [rad] % 0.5
TheConfig.SensorNoise.RDI.roll     = 0.5 * DTOR; % [rad] % 0.5
TheConfig.SensorNoise.RDI.u        = 0.005;      % [m/s] % 0.005
TheConfig.SensorNoise.RDI.v        = 0.005;      % [m/s]
TheConfig.SensorNoise.RDI.w        = 0.005;      % [m/s]
TheConfig.SensorNoise.XBOW.hr      = 0.25 * DTOR; % [rad/s] % 1.0
TheConfig.SensorNoise.XBOW.pr      = 0.25 * DTOR; % [rad/s]
TheConfig.SensorNoise.XBOW.rr      = 0.25 * DTOR; % [rad/s]
TheConfig.SensorNoise.PARO.depth   = 0.01;      % [m]   % 0.05
TheConfig.SensorNoise.PHINS.heading= 0.5 * DTOR; % [rad]
TheConfig.SensorNoise.PHINS.pitch  = 0.1 * DTOR; % [rad]
TheConfig.SensorNoise.PHINS.roll   = 0.1 * DTOR; % [rad]
TheConfig.SensorNoise.KVH.heading  = 2.0 * DTOR; % [rad]
TheConfig.SensorNoise.KVH.pitch    = 1.0 * DTOR; % [rad]
TheConfig.SensorNoise.KVH.roll     = 1.0 * DTOR; % [rad]
TheConfig.SensorNoise.LBL.nx       = 0.01;       % [m]
TheConfig.SensorNoise.LBL.ny       = 0.01;       % [m]


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
%TheConfig.SensorXform.RDI.Rvs = rotxyz([0,0,45]*DTOR); % eyeball approximate
TheConfig.SensorXform.RDI.tvs_v = [0.08, 0, 0.294]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.RDI.Tvs = ...
    [TheConfig.SensorXform.RDI.Rvs, TheConfig.SensorXform.RDI.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.RDI.x_vs = [TheConfig.SensorXform.RDI.tvs_v; ...
            		         rot2rph(TheConfig.SensorXform.RDI.Rvs)];

%======================================
% PHINS TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.PHINS.Rvs = rotxyz([180,0,90]*DTOR); % correct
%TheConfig.SensorXform.PHINS.Rvs = rotxyz([0,0,90]*DTOR);  % wrong
TheConfig.SensorXform.PHINS.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from phins to vehicle
TheConfig.SensorXform.PHINS.Tvs = ...
    [TheConfig.SensorXform.PHINS.Rvs, TheConfig.SensorXform.PHINS.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.PHINS.x_vs = [TheConfig.SensorXform.PHINS.tvs_v; ...
		                   rot2rph(TheConfig.SensorXform.PHINS.Rvs)];
% 6x6 covariance matrix
TheConfig.SensorXform.PHINS.P_vs = blkdiag(zeros(3),1.0*DTOR*eye(3)).^2;

%======================================
% XBOW TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.XBOW.Rvs = rotxyz([0,0,0]*DTOR);
%TheConfig.SensorXform.XBOW.Rvs = rotxyz([-90,0,-90]*DTOR);
TheConfig.SensorXform.XBOW.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from xbow to vehicle
TheConfig.SensorXform.XBOW.Tvs = ...
    [TheConfig.SensorXform.XBOW.Rvs, TheConfig.SensorXform.XBOW.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.XBOW.x_vs = [TheConfig.SensorXform.XBOW.tvs_v; ...
		                  rot2rph(TheConfig.SensorXform.XBOW.Rvs)];

%======================================
% KVH TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.KVH.Rvs = rotxyz([0,0,0]*DTOR);
TheConfig.SensorXform.KVH.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from kvh to vehicle
TheConfig.SensorXform.KVH.Tvs = ...
    [TheConfig.SensorXform.KVH.Rvs, TheConfig.SensorXform.KVH.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.KVH.x_vs = [TheConfig.SensorXform.KVH.tvs_v; ...
		                 rot2rph(TheConfig.SensorXform.KVH.Rvs)];

%======================================
% PARO TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.PARO.Rvs = rotxyz([0,0,0]*DTOR);
TheConfig.SensorXform.PARO.tvs_v = [-0.017, -0.3500, 0.2250]';
% 4x4 coordinate transformation from paro to vehicle
TheConfig.SensorXform.PARO.Tvs = ...
    [TheConfig.SensorXform.PARO.Rvs, TheConfig.SensorXform.PARO.tvs_v; ...
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
%TheConfig.SensorXform.PXF.Rvs = rotxyz([4.471 1.336 90.576]*DTOR); % EKF optimized
TheConfig.SensorXform.PXF.Rvs = rotxyz([4.471 0 90.576]*DTOR); % ryan temp
%TheConfig.SensorXform.PXF.Rvs = rotxyz([4 1 90.5]*DTOR); % empirically fudged
%TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 90]*DTOR);   % rough guess
% but if the camera was mounted "backwards" on the
% vehicle (i.e. rotated 180 from what it
% should have been) the rotation matrix is instead:
% TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 -90]*DTOR);

% vector from vehicle origin to camera origin expressed 
% in vehicle frame
TheConfig.SensorXform.PXF.tvs_v = [0.045, -0.17, 0.3935]'; % meters PXF camera
%TheConfig.SensorXform.tvc_v = [0.015, 0.15, 0.3585]'; % meters JHU camera
% 4x4 coordinate transformation from camera to vehicle
TheConfig.SensorXform.PXF.Tvs = ...
    [TheConfig.SensorXform.PXF.Rvs, TheConfig.SensorXform.PXF.tvs_v; ...
     0  0  0  1];
% 6x1 pose vector
TheConfig.SensorXform.PXF.x_vs = [TheConfig.SensorXform.PXF.tvs_v; ...
		                 rot2rph(TheConfig.SensorXform.PXF.Rvs)];
% 6x6 covariance matrix
%TheConfig.SensorXform.PXF.P_vs = blkdiag(0.001*eye(3),5.0*DTOR*eye(3)).^2;

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
TheConfig.SensorXform.Twl = ...
    [TheConfig.SensorXform.Rwl, TheConfig.SensorXform.twl_w; ...
     0  0  0  1];
% 6x1 pose vector
TheConfig.SensorXform.x_wl = [TheConfig.SensorXform.twl_w; ...
		       rot2rph(TheConfig.SensorXform.Rwl)];

%======================================
% HEURISTIC SENSOR OUTLIER REJECTION
%======================================
% RDI
TheConfig.Heuristic.RDI_e_filt             = off;
TheConfig.Heuristic.RDI_num_beams          = on;
TheConfig.Heuristic.RDI_zero_velocity_filt = on;
TheConfig.Heuristic.RDI_uvw_flier_filt     = off;
TheConfig.Heuristic.RDI_rph_flier_filt     = off;
TheConfig.Heuristic.RDI_fix_srange         = on;
% XBOW
TheConfig.Heuristic.XBOW_fix_parse = on;
TheConfig.Heuristic.XBOW_fix_timebase = off;
TheConfig.Heuristic.XBOW_fake_from_phins = off;
% PARO
TheConfig.Heuristic.PARO_serial_filt = off;


%======================================
% FEATURE VECTOR SPECIFIC
%======================================
% size of feature window [(2w+1)x(2w+1)]
TheConfig.ImageFeature.w = 16;
% discretized Zernike patch size and moment order
TheConfig.Zbasis = zernikeBasisFcnsPolar([128,32],24);
% number of image feature regions (for harris & clahs)
TheConfig.ImageFeature.tile = [8 10];
% clahs params (used for SIFT features)
TheConfig.ImageFeature.clahsArgs = ...
    {'Distribution','rayleigh','NumTiles',[8 10],'ClipLimit',7.5e-3,'NBins',2^10};
% number of feature points to extract
TheConfig.ImageFeature.Nmax = 1000;
% minimum number of points to register
TheConfig.ImageFeature.MIN_NUM_PTS = 15;
% feature vector similarity threshold (normalized correlation score)
% normalized correlation threshold is related to normalized SSD via:
% thresh_SSD = 2(1-thresh_NC)
% thresh_NC  = 1-thresh_SSD/2
TheConfig.ImageFeature.simthresh = 0.75;

%======================================
% PLOT SELECTION
%======================================
TheConfig.Plot.plot_nu_t       = off;
TheConfig.Plot.plot_ssv        = off;
TheConfig.Plot.plot_links_xy   = on;
TheConfig.Plot.plot_corrcoeff  = off;
TheConfig.Plot.harris_pts      = off;
TheConfig.Plot.sift_pts        = off;
TheConfig.Plot.rdi_image_bathy = off;
TheConfig.Plot.twoview_pt_xfer = on;
TheConfig.Plot.putative_corr   = off;
TheConfig.Plot.plot_stats      = off;
% two_view_register
TheConfig.Plot.inlier_motion   = on;
TheConfig.Plot.outlier_motion  = off;
TheConfig.Plot.summary_motion  = on;
TheConfig.Plot.sample_epipolar_lines = on; %
TheConfig.Plot.mahal_pose      = off;
TheConfig.Plot.mle_pose        = on;
TheConfig.Plot.mle_scene       = on; %
TheConfig.Plot.mle_pose_scene  = on; %

TheConfig.Plot.plinks          = off;
TheConfig.Plot.vlinks          = on;
TheConfig.Plot.CIfusion        = off;
