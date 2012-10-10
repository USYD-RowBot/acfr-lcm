%JUN 05 2004  Titanic dataset using Hercules on Ballard Cruise

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
TheConfig.Data.navDir = '/files1/data/oc387/Log/d20030325/';
TheConfig.Data.navFile = '20030325_1916.SEABED.syscfg';
TheConfig.Data.navLoader = @loadseabed;

% image directory
%TheConfig.Data.imageDir  = '/files1/data/oc387/Images/oc387-6boulder/ColorConvert/';
TheConfig.Data.imageDir  = '/files1/data/oc387/Images/oc387-6boulder/ColorCorrected/';


% image extentions [e.g. tif jpg png etc]
%TheConfig.Data.imageExtension = 'png';
TheConfig.Data.imageExtension = 'jpg';

% camera calibration file & image size
TheConfig.Data.imageCalibrationFile = '/files1/data/oc387/Color_Calib_Results_20021125.m';
TheConfig.Calib = initializeCalib(TheConfig.Data.imageCalibrationFile);

% rdi compass bias file
% specify an empty matrix if no file exists
TheConfig.Data.headingBiasFile = '/files1/thesis/van/fcns_headingBias/compass_bias.op.mat';

% image sequence to processs
%TheConfig.Data.imageSequence = [729:730];
TheConfig.Data.imageSequence = [758:759];
%TheConfig.Data.imageSequence = [1239:1240];
%TheConfig.Data.imageSequence = [700:800];
TheConfig.Data.imgest = TheConfig.Data.imageSequence;
% 140:220 turning north corner #1
% 331:411 turning south corner #1
% 520:600 turning north corner #2
% 705:785 turning south corner #2
% 885:965 turning north corner #3
% 940:955 dvl dropout 
% 732,756 Crazy structure!!! Must have wrong solution, giving up...

% number of images in sequence
TheConfig.Data.nImages = length(TheConfig.Data.imgest);

% output data directory
TheConfig.Data.outdir = '/files1/processed/van/output/oc387/';

% memory stack size for Image Features
TheConfig.Data.stackSize = 10;

% max survey altitude
TheConfig.Data.maxAltitude = 3.75; %[m]

% user interaction
TheConfig.Data.vpause = 1.5;  % notify time in seconds

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
TheConfig.Estimator.record_ssv           = off;
TheConfig.Estimator.ssvSampleHertz       = 2;
TheConfig.Estimator.ssvSamplePeriod      = 1/TheConfig.Estimator.ssvSampleHertz;
TheConfig.Estimator.max_link_hypoth      = 5;
TheConfig.Estimator.useSavedCameraMeas   = 0; %0 - don't use
                                              %1 - use if available
					      %2 - use if available and skip pairs w/o saved measurement

%=======================================
% CHOOSE SENSOR OBSERVATION MODEL
%=======================================
TheConfig.ObservationModel.RDI   = @om_rdi_uvwrph;
TheConfig.ObservationModel.XBOW  = @om_xbow_abc;
TheConfig.ObservationModel.PHINS = [];%@om_octans_rph;
TheConfig.ObservationModel.PARO  = @om_paro_z;
TheConfig.ObservationModel.PXF   = @om_camera_Rae21;

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
TheConfig.ProcessModel.Noise.uvw = 0.01  * [1,1,1/4];        % m/s 0.01
%TheConfig.ProcessModel.Noise.uvw = 0.5  * [1,1,1/4];        % m/s 0.01
TheConfig.ProcessModel.Noise.abc = 0.160  * DTOR * [1,1,1];   % rad/s 0.10
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
TheConfig.SensorNoise.RDI.u        = 0.01;      % [m/s] % 0.003
TheConfig.SensorNoise.RDI.v        = 0.01;      % [m/s]
TheConfig.SensorNoise.RDI.w        = 0.01;      % [m/s]
TheConfig.SensorNoise.XBOW.hr      = 0.25 * DTOR; % [rad/s] % 1.0
TheConfig.SensorNoise.XBOW.pr      = 0.25 * DTOR; % [rad/s]
TheConfig.SensorNoise.XBOW.rr      = 0.25 * DTOR; % [rad/s]
TheConfig.SensorNoise.PARO.depth   = 0.02;       % [m]   % 0.05
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
TheConfig.SensorXform.RDI.Rvs = rotxyz([0,0,0]*DTOR); % default
TheConfig.SensorXform.RDI.tvs_v = [0, 0, 0]';
% 4x4 coordinate transformation from doppler to vehicle
TheConfig.SensorXform.RDI.Tvs = ...
    [TheConfig.SensorXform.RDI.Rvs, TheConfig.SensorXform.RDI.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.RDI.x_vs = [TheConfig.SensorXform.RDI.tvs_v; ...
		                 rot2rph(TheConfig.SensorXform.RDI.Rvs)];


%======================================
% XBOW TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.XBOW.Rvs = rotxyz([0,0,0]*DTOR);
TheConfig.SensorXform.XBOW.tvs_v = [0,0,0]';
% 4x4 coordinate transformation from xbow to vehicle
TheConfig.SensorXform.XBOW.Tvs = ...
    [TheConfig.SensorXform.XBOW.Rvs, TheConfig.SensorXform.XBOW.tvs_v; ...
     0 0 0 1];
% 6x1 pose vector
TheConfig.SensorXform.XBOW.x_vs = [TheConfig.SensorXform.XBOW.tvs_v; ...
		                  rot2rph(TheConfig.SensorXform.XBOW.Rvs)];

%======================================
% PARO TO VEHICLE
%--------------------------------------
TheConfig.SensorXform.PARO.Rvs = rotxyz([0,0,0]*DTOR);
TheConfig.SensorXform.PARO.tvs_v = [0, 0, 0]';
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
TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 90]*DTOR);   % rough guess
% but if the camera was mounted "backwards" on the
% vehicle (i.e. rotated 180 from what it
% should have been) the rotation matrix is instead:
% TheConfig.SensorXform.PXF.Rvs = rotxyz([0 0 -90]*DTOR);

% vector from vehicle origin to camera origin expressed 
% in vehicle frame
TheConfig.SensorXform.PXF.tvs_v = [1.4, 0, 0]'; % meters PXF camera
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
TheConfig.Heuristic.XBOW_fix_parse       = on;
TheConfig.Heuristic.XBOW_fix_timebase    = off;
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
% number of image feature regions
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
TheConfig.Plot.harris_pts      = on;
TheConfig.Plot.sift_pts        = on;
TheConfig.Plot.rdi_image_bathy = on;
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
