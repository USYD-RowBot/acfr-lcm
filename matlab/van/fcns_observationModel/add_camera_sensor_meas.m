function [meas_t,regflag] = add_camera_sensor_meas(nav_t,fni,fnj,x_lv1,x_lv2,x_vc,Sigma,TheConfig);
%function [meas_t,regflag] = add_camera_sensor_meas(nav_t,fni,fnj,x_lv1,x_lv2,x_vc,Sigma,TheConfig);
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-20-2003      rme         Created.
%    01-07-2004      rme         Modified to register feature pair (fni,fnj)
%    03-31-2004      rme         changed arguments to camera_pose_t_struct.m
%    04-01-2004      rme         changed relview_ptxfer to accept vector
%                                of Z1 and Cov_Z1
%    04-06-2004      rme         Changed to use om_camera_Rae21
%    04-10-2004      rme         Renamed from camera_meas.m to add_cam_sensor_meas.m.
%                                Updated to output a meas_t structure.
%    09-01-2004      rme         Modified to use TheConfig.slam field
%    09-08-2004      rme         Changed TheConfig.slam to TheConfig.Estimator
%    10-28-2004      rme         Major code reorganization.
%    11-03-2004      rme         Modified what gets stuffed into meas_t.varargin
%    11-04-2004      rme         Renamed to add_camera_sensor_meas.m
%    11-05-2004      rme         Fixed argument typo in generating Features2
%    11-05-2004      rme         Fixed bug in calculation of covariance P_c2c1
%    11-05-2004      rme         Changed to pass in pose estimates as arguments.
%    11-14-2004      rme         Changed pixeltol radius from 5 to 100 pixels
%    12-18-2004      rme         Added SIFT and Zernike features
%    12-20-2004      rme         Added putative corr summary plot
%    12-20-2004      rme         Moved Bathy structure out from under the TheJournal
%                                and into a new global variable TheBathy
%    12-21-2004      rme         Added ability to load a previously saved image pair correspondence file
%    12-22-2004      rme         Updated P_c1c2, P_c2c1 calculation to assume x_vc is static.
%    01-18-2005      rme         Added capability to process saved camera measurements.
%    01-22-2006      rme         Added capability to process manual correspondences.
%    02-07-2006      rme         Added check for TheConfig.Estimator.useManualCorrespondences
%                                for backwards config file compatability.

global TheJournal TheBathy;

% init ouput args
meas_t = []; regflag = false;

%====================================================
% LOAD SAVE CAMERA MEASUREMENT
%====================================================
if TheConfig.Estimator.useSavedCameraMeas;
  % check for saved correspondence file and load it
  Inliers = private_loadcorr(TheJournal.Index.featureLUT(fni), ...
			     TheJournal.Index.featureLUT(fnj), ...
			     TheConfig);
  if ~isempty(Inliers); % return saved camera measurement
    regflag = true;
    % store camera measurement observation model function handle
    meas_t.omfhandle = @om_camera_aerph_Rae21;
    meas_t.omfhandleString = func2str(meas_t.omfhandle);
    % set delayed state measurement flag
    meas_t.isaDelayedStateMeas = true;
    % store camera relative pose measurement and covariance
    meas_t.z = Inliers.z;
    meas_t.R = Inliers.R;
    if TheConfig.Estimator.estimateCameraXform;
      meas_t.varargin = {x_vc_i,TheConfig.Estimator.estimateCameraXform};
    else;
      meas_t.varargin = {TheConfig.SensorXform.PXF.x_vs,TheConfig.Estimator.estimateCameraXform};
    end;
    % store augmented state feature indexes
    Xp_i = TheJournal.Index.Xp_i;
    meas_t.fni = fni;
    meas_t.fnj = fnj;
    meas_t.Xfi_i = TheJournal.Index.Xf_ii{fni}(Xp_i);
    meas_t.Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);
    % add this image pair to the list of verified image matches
    TheJournal.Links.vlinks(fni,fnj) = 1;
    % add this pair to the state adjacency graph
    TheJournal.Links.fgraph(fni,fnj) = 1;
    TheJournal.Links.fgraph(fnj,fni) = 1;
    return;
  elseif TheConfig.Estimator.useSavedCameraMeas==2; % skip pairs w/o save correspodence file
    return;
  end;
end;

%====================================================
% LOAD IMAGE DATA
%====================================================
to = clock;
% featureLUT is a feature look-up-table of image numbers organized feature number
Image1 = initializeImage(TheJournal.Index.featureLUT(fni),TheConfig);
Image2 = initializeImage(TheJournal.Index.featureLUT(fnj),TheConfig);

% print to screen
setTerminal('normal');
fprintf('\n');
fprintf('Attempting to register hypothesized image pair: (%d,%d)\n',fni,fnj);
fprintf('%s  %s\n',Image1.filename,Image2.filename);
fprintf('-------------------------------------------------------------------------\n');
setTerminal('restore');


% predicted relative poses and 1st order covariances assuming
% Sigma elements are arranged in the order: [x_lv1,x_lv2,x_vc]
%---------------------------------------------------
% camera 2 w.r.t camera 1
[x_c1c2,J] = relative_sensor_pose(x_lv1,x_lv2,x_vc);
P_c1c2 = spdproduct(Sigma(1:12,1:12),J(:,1:12)');
% camera 1 w.r.t camera 2
[x_c2c1,J] = relative_sensor_pose(x_lv2,x_lv1,x_vc);
J = J(:,[7:12,1:6,13:18]); % re-arrange jacobian to match order of Sigma
P_c2c1 = spdproduct(Sigma(1:12,1:12),J(:,1:12)');


%====================================================
% LOAD/GENERATE IMAGE FEATURE DATA
%====================================================
% transfer absolute vehicle pose to absolute camera pose
%--------------------------------------------------------
x_lc1 = head2tail(x_lv1,x_vc); % 6 DOF camera state at time t1
x_lc2 = head2tail(x_lv2,x_vc); % 6 DOF camera state at time t2

% return a feature structure containing interest points and descriptors
Features1 = initializeFeatures(Image1,x_lc1,TheConfig);
Features2 = initializeFeatures(Image2,x_lc2,TheConfig);

% plot harris interest points
if TheConfig.Plot.harris_pts;
  figure(20); clf;
  imagesc(TheConfig.Calib.udata,TheConfig.Calib.vdata,Image1.Iwarp); colormap gray; axis image off;
  hold on; plot(Features1.Zernike.uc,Features1.Zernike.vc,'y+'); hold off;
  set(20,'Name','Harris Points I1');
  figure(21); clf;
  imagesc(TheConfig.Calib.udata,TheConfig.Calib.vdata,Image2.Iwarp); colormap gray; axis image off;
  hold on; plot(Features2.Zernike.uc,Features2.Zernike.vc,'y+'); hold off; colormap gray;
  set(21,'Name','Harris Points I2');
end;

% plot sift interest points
if TheConfig.Plot.sift_pts;
  figure(22); clf; drawsift(Image1.Iwarp,Features1.Sift.keys,2); set(22,'Name','SIFT Points I1');
  figure(23); clf; drawsift(Image2.Iwarp,Features2.Sift.keys,2); set(23,'Name','SIFT Points I2');
end;

% prior knowledge on scene depth
%----------------------------------------------------
% RDI measured bathymetry structure associated with each image
bathy1_t = TheBathy(fni);
bathy2_t = TheBathy(fnj);

% plot RDI bathymetry points
if TheConfig.Plot.rdi_image_bathy;
  udata = TheConfig.Calib.udata;
  vdata = TheConfig.Calib.vdata;
  figure(24); clf; plotbathy(udata,vdata,Image1,bathy1_t); set(24,'Name','RDI Bathymetry I1');
  figure(25); clf; plotbathy(udata,vdata,Image2,bathy2_t); set(25,'Name','RDI Bathymetry I2');
end;

% assign each image feature point a scene depth prior
%----------------------------------------------------
pixeltol = 100;
% zernike features
[Features1.Zernike.Z,Features1.Zernike.Cov_Z] = ...
    feature_depth_prior(Features1.Zernike.uc, Features1.Zernike.vc, ...
			bathy1_t.uc, bathy1_t.vc, bathy1_t.Z, bathy1_t.Cov_Z, pixeltol);
[Features2.Zernike.Z,Features2.Zernike.Cov_Z] = ...
    feature_depth_prior(Features2.Zernike.uc, Features2.Zernike.vc, ...
			bathy2_t.uc, bathy2_t.vc, bathy2_t.Z, bathy2_t.Cov_Z, pixeltol);
% sift features
[Features1.Sift.Z,Features1.Sift.Cov_Z] = ...
    feature_depth_prior(Features1.Sift.uc, Features1.Sift.vc, ...
			bathy1_t.uc, bathy1_t.vc, bathy1_t.Z, bathy1_t.Cov_Z, pixeltol);
[Features2.Sift.Z,Features2.Sift.Cov_Z] = ...
    feature_depth_prior(Features2.Sift.uc, Features2.Sift.vc, ...
			bathy2_t.uc, bathy2_t.vc, bathy2_t.Z, bathy2_t.Cov_Z, pixeltol);

% print prior pose information
%----------------------------------------------------
fprintf('\nRelative Pose Prior:\n');
print_pose_prior(x_c1c2,P_c1c2,'12',1,0);
print_pose_prior(x_c2c1,P_c2c1,'21',0,1);
fprintf('Depth Prior: Z1 = %.2f+/-%.2f,  Z2 = %.2f+/-%.2f\n', ...
	median(Features1.Zernike.Z),sqrt(median(Features1.Zernike.Cov_Z)), ...
	median(Features2.Zernike.Z),sqrt(median(Features2.Zernike.Cov_Z)));
fprintf('\n');


%================================================================================
% ASSIGN PUTATIVE CORRESPONDENCES BASED UPON PRIOR POSE AND FEATURE SIMILARITY
%================================================================================
% zernike feature similarity matrix (normalized correlation)
smatrixZernike = zernikeCorrScorePolar(Features1.Zernike.A_nm, Features2.Zernike.A_nm, TheConfig.Zbasis);
% zernike putative correspondence assignment
% note: we want to *maximize* normalized correlation score
[pselZernike1,pselZernike2,simZernikeA,simZernikeB] = ...
    putative_corrset(Features1.Zernike.uc, Features1.Zernike.vc, ...
		     Features2.Zernike.uc, Features2.Zernike.vc, ...
		     Features1.Zernike.Z,  Features2.Zernike.Z, ...
		     Features1.Zernike.Cov_Z, Features2.Zernike.Cov_Z, ...
		     smatrixZernike, 'max', x_c1c2, x_c2c1, P_c1c2, P_c2c1, ...
		     Image1, Image2, TheConfig);
% keep only those with a normalized correlation score above threshold
% and whose second best match is below threshold
%tmp = find(simZernikeA >= TheConfig.ImageFeature.simthresh);
ncthresh = TheConfig.ImageFeature.simthresh; % norm corr threshold
tmp = find( (simZernikeA >= ncthresh) & (simZernikeB < ncthresh) );
pselZernike1 = pselZernike1(tmp); pselZernike2 = pselZernike2(tmp);
simZernikeA = simZernikeA(tmp); simZernikeB = simZernikeB(tmp);

% sift feature similarity matrix (squared Euclidean distance)
smatrixSift = siftSimilarityScore(Features1.Sift.keys,Features2.Sift.keys);
% sift putative correspondence assignment
% note: we want to *minimize* sift Euclidean distance
[pselSift1,pselSift2,simSiftA,simSiftB] = ...
    putative_corrset(Features1.Sift.uc, Features1.Sift.vc, ...
		     Features2.Sift.uc, Features2.Sift.vc, ...
		     Features1.Sift.Z,  Features2.Sift.Z, ...
		     Features1.Sift.Cov_Z, Features2.Sift.Cov_Z, ...       
		     smatrixSift, 'min', x_c1c2, x_c2c1, P_c1c2, P_c2c1, ...
		     Image1, Image2, TheConfig);
% keep matches which are "far enough away" from second best match.
% 0.6 is recommended by Lowe, but we can relax this constraint a bit
% since we have nav and are using a robust registration method like RANSAC so we
% can deal with outliers.
tmp = find(simSiftA < 0.7^2*simSiftB); % 0.6 is recommended by Lowe
pselSift1 = pselSift1(tmp); pselSift2 = pselSift2(tmp);
simSiftA = simSiftA(tmp); simSiftB = simSiftB(tmp);

% sift can assign "multiple" descriptors in scale-space to the same feature point.
% reduce the putative correspondence set to a unique set.
[junk,tmp] = unique([Features1.Sift.uc(pselSift1), Features1.Sift.vc(pselSift1), ...
		    Features2.Sift.uc(pselSift2), Features2.Sift.vc(pselSift2)],'rows');
pselSift1 = pselSift1(tmp); pselSift2 = pselSift2(tmp);
simSiftA = simSiftA(tmp); simSiftB = simSiftB(tmp);

%=======================================================
% CONCATENATE FEATURES
%=======================================================
% cat interest points
uc1 = [Features1.Zernike.uc; Features1.Sift.uc]; vc1 = [Features1.Zernike.vc; Features1.Sift.vc];
uc2 = [Features2.Zernike.uc; Features2.Sift.uc]; vc2 = [Features2.Zernike.vc; Features2.Sift.vc];
% cat scene depth prior
Z1  = [Features1.Zernike.Z;  Features1.Sift.Z];   Z2 = [Features2.Zernike.Z;  Features2.Sift.Z];

% cat putative correspondence indices
psel1 = [pselZernike1; Features1.Zernike.Nf+pselSift1];
psel2 = [pselZernike2; Features2.Zernike.Nf+pselSift2];

% user defined minimum number of point correspondences
MinNumPts = TheConfig.ImageFeature.MIN_NUM_PTS;

%=======================================================
% CHECK FOR MANUAL POINT OVERRIDE
%=======================================================
ManualOverride = false;
if isfield(TheConfig.Estimator,'useManualCorrespondences') && ...
      (TheConfig.Estimator.useManualCorrespondences==true);
  % check for saved correspondence file and load it
  ManualPts = private_loadmanual(TheJournal.Index.featureLUT(fni), ...
				 TheJournal.Index.featureLUT(fnj), ...
				 TheConfig);
  if ~isempty(ManualPts);
    ManualOverride = true;
    [uc1,vc1,uc2,vc2] = deal(ManualPts.uc1,ManualPts.vc1,ManualPts.uc2,ManualPts.vc2);
    [psel1,psel2] = deal([1:length(uc1)]');
    MinNumPts = 6;
  end;
end;


%=======================================================
% TRY TO REGISTER IMAGES BASED UPON CALIBRATED CAMERA
% EPIPOLAR MODEL
%=======================================================    
if (length(psel1) >= MinNumPts);
  % try to measure relative pose between image pair
  [regflag,p_21,Cov_21,isel1,isel2] = ...
      twoview_register(uc1, vc1, uc2, vc2, psel1, psel2, x_c2c1, P_c2c1, Z1, Z2, ...
		       TheConfig.Calib.K, MinNumPts, Image1.Iwarp, Image2.Iwarp, TheConfig);
  % outlier index
  osel1 = setdiff(psel1,isel1);
  osel2 = setdiff(psel2,isel2);
else;
  % unable to register image pair, display message to user
  setTerminal('warning');
  fprintf('***%s: Only %d putative matches found, min required is %d.\n', ...
	  mfilename,length(psel1),MinNumPts);
  setTerminal('restore');
end;

% if we were successful in making a camera based relative pose
% measurement, add it to the list
if regflag == true;  
  % store camera measurement observation model function handle
  meas_t.omfhandle = @om_camera_aerph_Rae21;
  meas_t.omfhandleString = func2str(meas_t.omfhandle);
  % set delayed state measurement flag
  meas_t.isaDelayedStateMeas = true;
  % store camera relative pose measurement and covariance
  meas_t.z = p_21;
  meas_t.R = Cov_21;
  if TheConfig.Estimator.estimateCameraXform;
    meas_t.varargin = {x_vc_i,TheConfig.Estimator.estimateCameraXform};
  else;
    meas_t.varargin = {TheConfig.SensorXform.PXF.x_vs,TheConfig.Estimator.estimateCameraXform};
  end;
  % store augmented state feature indexes
  Xp_i = TheJournal.Index.Xp_i;
  meas_t.fni = fni;
  meas_t.fnj = fnj;
  meas_t.Xfi_i = TheJournal.Index.Xf_ii{fni}(Xp_i);
  meas_t.Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);
  % add this image pair to the list of verified image matches
  TheJournal.Links.vlinks(fni,fnj) = 1;
  % add this pair to the state adjacency graph
  TheJournal.Links.fgraph(fni,fnj) = 1;
  TheJournal.Links.fgraph(fnj,fni) = 1;

  if ManualOverride;
    % print results
    textcolor('dim','yellow','black');
    fprintf('\n==>Manual Summary: Inliers %d\tOutliers %d\n', ...
	    length(isel1),length(osel1));
    setTerminal('restore');
  else;
    % putative breakdown zernike
    iselZernike1 = isel1(isel1<=Features1.Zernike.Nf); % inliers
    iselZernike2 = isel2(isel2<=Features2.Zernike.Nf);
    oselZernike1 = osel1(osel1<=Features1.Zernike.Nf); % outliers
    oselZernike2 = osel2(osel2<=Features2.Zernike.Nf);
    % putative breakdown sift
    iselSift1 = isel1(isel1 > Features1.Zernike.Nf) - Features1.Zernike.Nf; % inliers
    iselSift2 = isel2(isel2 > Features2.Zernike.Nf) - Features2.Zernike.Nf;
    oselSift1 = osel1(osel1 > Features1.Zernike.Nf) - Features1.Zernike.Nf; % outliers
    oselSift2 = osel2(osel2 > Features2.Zernike.Nf) - Features2.Zernike.Nf;
    % print results
    textcolor('dim','yellow','black');
    fprintf('\n==>Putative Summary: Inliers %d/%d\tOutliers %d/%d\tZernike/SIFT\n', ...
	    length(iselZernike1),length(iselSift1),length(oselZernike1),length(oselSift1));
    setTerminal('restore');
    % plot results
    if TheConfig.Plot.summary_motion;
      private_plot_summary(iselZernike1,iselZernike2,iselSift1,iselSift2, ...
			   Features1,Features2,Image1,Image2);
    end;

    % write to disk camera measurement & correspondence set
    private_savecorr(meas_t,iselZernike1,iselZernike2,iselSift1,iselSift2,Image1,Image2,TheConfig);
  end;
else;
  % add this image pair to the list of verfied non-matches
  TheJournal.Links.vlinks(fni,fnj) = TheJournal.Links.vlinks(fni,fnj) - 1;
  setTerminal('warning');
  fprintf('***%s: Unable to register image pair\n',mfilename);
  setTerminal('restore');
end;

fprintf('_________________________________________________________________________\n');
fprintf('==>%s: etime %.2fs\n',mfilename,etime(clock,to));
fprintf('=========================================================================\n');


%************************************************************************************
function private_savecorr(meas_t,iselZernike1,iselZernike2,iselSift1,iselSift2,Image1,Image2,TheConfig);

% check if directory exists
outdir = strcat(TheConfig.Data.outdir,'proc/corrset/');
if ~exist(outdir,'dir');
  mkdir(TheConfig.Data.outdir,'proc/corrset');
end;

% save correspondences
Inliers.imgnum1      = Image1.imgnum;
Inliers.imgnum2      = Image2.imgnum;
Inliers.z            = meas_t.z;
Inliers.R            = meas_t.R;
Inliers.iselZernike1 = iselZernike1;
Inliers.iselZernike2 = iselZernike2;
Inliers.iselSift1    = iselSift1;
Inliers.iselSift2    = iselSift2;
filename = sprintf('Inliers-%04d-%04d.mat',Image1.imgnum,Image2.imgnum);
save([outdir,filename],'Inliers');

%************************************************************************************
function Inliers = private_loadcorr(imgnum1,imgnum2,TheConfig);

% check if file exists
outdir = strcat(TheConfig.Data.outdir,'/proc/corrset/');
filename = sprintf('Inliers-%04d-%04d.mat',imgnum1,imgnum2);
if exist([outdir,filename],'file');
  % load correspondences
  setTerminal('highlight');
  fprintf('==>%s: Loading %s... ',mfilename,filename);
  tmp = load([outdir,filename]);
  Inliers = tmp.Inliers;
  fprintf('done\n');
  setTerminal('restore');
else;
  Inliers = [];
end;


%************************************************************************************
function ManualPts = private_loadmanual(imgnum1,imgnum2,TheConfig);

% check if file exists
outdir = strcat(TheConfig.Data.outdir,'manual/');
filename = sprintf('Manual-%04d-%04d.dat',imgnum2,imgnum1);
if exist([outdir,filename],'file');
  % load correspondences
  setTerminal('highlight');
  fprintf('==>%s: Loading %s... ',mfilename,filename);
  tmp = load([outdir,filename]);
  [u2,v2] = deal(tmp(:,1),tmp(:,2));
  [u1,v1] = deal(tmp(:,3),tmp(:,4));
  % calculate distortion corrected feature points
  tmp = tformfwd([u1,v1],TheConfig.Calib.TformRemoveDistortion);
  [uc1,vc1] = deal(tmp(:,1),tmp(:,2));
  tmp = tformfwd([u2,v2],TheConfig.Calib.TformRemoveDistortion);
  [uc2,vc2] = deal(tmp(:,1),tmp(:,2));
  fprintf('done\n');
  setTerminal('restore');
  ManualPts.u1 = u1; ManualPts.uc1 = uc1;
  ManualPts.v1 = v1; ManualPts.vc1 = vc1;
  ManualPts.u2 = u2; ManualPts.uc2 = uc2;
  ManualPts.v2 = v2; ManualPts.vc2 = vc2;
else;
  ManualPts = [];
end;


%**************************************************************************************
function private_plot_summary(iselZernike1,iselZernike2,iselSift1,iselSift2, ...
			      Features1,Features2,Image1,Image2);
% plot results
figure(56);
plot_motion(Image1.Iwarp,...
	    Features1.Zernike.uc(iselZernike1), Features1.Zernike.vc(iselZernike1), ...
	    Features2.Zernike.uc(iselZernike2), Features2.Zernike.vc(iselZernike2), ...
	    [],{'Color','g'});
plot_motion([],...
	    Features1.Sift.uc(iselSift1), Features1.Sift.vc(iselSift1), ...
	    Features2.Sift.uc(iselSift2), Features2.Sift.vc(iselSift2), ...
	    [],{'Color','c'});
string = sprintf('Putative Inlier Summary I1: %d/%d Zernike/SIFT', ...
		 length(iselZernike1),length(iselSift1));
title(string);
set(56,'Name','Putative Inlier Summary I1');

figure(57);
plot_motion(Image2.Iwarp,...
	    Features2.Zernike.uc(iselZernike2), Features2.Zernike.vc(iselZernike2), ...
	    Features1.Zernike.uc(iselZernike1), Features1.Zernike.vc(iselZernike1), ...
	    [],{'Color','g'});
plot_motion([],...
	    Features2.Sift.uc(iselSift2), Features2.Sift.vc(iselSift2), ...
	    Features1.Sift.uc(iselSift1), Features1.Sift.vc(iselSift1), ...
	    [],{'Color','c'});
string = sprintf('Putative Inlier Summary I2: %d/%d Zernike/SIFT', ...
		 length(iselZernike1),length(iselSift1));
title(string);
set(57,'Name','Putative Inlier Summary I2');

