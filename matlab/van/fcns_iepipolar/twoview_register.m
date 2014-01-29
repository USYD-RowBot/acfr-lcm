function [regflag,p_21_mle_5dof,Cov_21_mle_5dof,isel1,isel2,X_mle] = ...
    twoview_register(u1,v1,u2,v2,psel1,psel2,pp_21,Cov_pp21,Z1,Z2,K,MIN_NUM_PTS,I1,I2,TheConfig)
%TWOVIEW_REGISTER
%   INPUTS: (u1,v1,u2,v2,psel1,psel2,pp_21,Cov_pp21,Z1,CovZ1,K,MIN_NUM_PTS,I1,I2,TheConfig)
%   ---------------------------------------------------------------------
%   u1,v1,u2,v2 are [N x 1] vectors of radially corrected feature points
%   in the radially corrected images I1,I2.  Note that the top left pixel
%   of I1 and I2 is defined to be (0,0).  psel1 and psel2 are [M x 1]
%   index column vectors defining putative correspondences between
%   (u1,v1) and (u2,v2).  pp_21 and Cov_pp21 define the relative pose
%   prior of camera 1 w.r.t. camera 2.  i.e. camera projection matrices
%   are assumed to be of the form:    P1 = [I | 0]   P2 = [R | t]
%   The pose prior is assumed to have been derived from nav and the mean
%   vector pp_21 = [tx, ty, tz, r, p, h]'.  K is the [3 x 3] camera
%   calibration matrix.  I1 and I2 are the radially corrected images used
%   for display purposed only.
%
%   OUTPUTS: [p_21_mle_5dof,Cov_21_mle_5dof,isel1,isel2,X_mle]
%   ---------------------------------------------------------------------  
%   p_21_mle_5dof and Cov_21_mle_5dof correspond respectively to the image
%   space measurement of relative pose and the associated uncertainty in the
%   measurement.  p_21_mle_5dof is a [5 x 1].  isel1 and isel2 are [Q x 1]
%   vectors of "inlier" correspondences. X_mle is an optional [3 x Q] 
%   matrix of bundle adjusted 3D triangulated feature points scaled to
%   unit baseline magnitude.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-03-2003      rme         Created.
%    09-26-2003      rme         Completed writing most of the function
%                                and added the help header above.
%                                Rewrote input argument specs.
%    10-02-2003      rme         When representing baseline as a unit
%                                magnitude direction vector, added code
%                                to force covariance matrix to be
%                                singular via analytical calculation of
%                                Jacobian in unitize.m
%    11-10-2003      rme         Added multi-color triangulated
%                                points to camera/scene plots
%    01-03-2004      rme         Fixed MLE epipolar plot to use MLE image
%                                points from projected optimal 3D structure
%    01-21-2004      rme         Added optional output of 3D feature points.
%    03-08-2004      rme         Added check for points triangulated
%                                behind camera.  Throw those points out.
%    03-23-2004      rme         Added mahalnobis distance threshold check.
%    04-06-2004      rme         Changed to direction vector representation
%                                to az,el instead of unit direction
%                                vector.  Also added unrapping of rph_ambig.
%    04-12-2004      rme         Updated to use funwrap.m
%    04-14-2004      rme         Added MIN_NUM_PTS argument.
%    09-09-2004      rme         Added TheConfig argument to control plots
%    11-05-2004      rme         Removed motion center outlier rejection code.
%    11-08-2004      rme         Added outlier rejection based upon triangulated scene points and 
%                                baseline scale set by pose prior
%    11-08-2004      rme         Major code restructuring.
%    11-09-2004      rme         Added triangulation constraint to MLE
%    11-18-2004      rme         Added heuristic to "inflate" camera measurement covariance
%                                based upon the number of inliers.
%    11-26-2004      rme         Twiddled with fprintf statements.
%    11-27-2004      rme         Print MLE Mahalanobius distance to screen.
%    12-21-2004      rme         Changed MAX_DIST from 1.5 to 2.0 times max([Z1;Z2]);
%    12-22-2004      rme         Added printing of nav uncertainty in 5 DOF measurement.
%                                Raised camera meas inflation factor from 3 to 4.
%                                Changed MAX_DIST to be based upon median rather than max.
%    12-22-2004      rme         Linearly scale cam meas covariance based upon number of inliers.
%    12-30-2004      rme         Added ability to use nav prior as initial guess.
%    01-01-2005      rme         Added calculation of baseline magnitude covariance.
%    01-02-2005      rme         Added Mahalanobius distance calcuation for MLE solution
%                                to compare nav prior to camera measurement along with
%                                vice-versa.  Was rejecting too many noise camera measurements
%                                because nav uncertainty of rph is very small.  It makes 
%                                sense to consider the precision of the camera measurement and nav
%                                for accepting/rejecting it when compared to nav.
%    03-12-2005      rme         Limit MAX_DIST to TheConfig.Data.maxAltitude
%    01-22-2006      rme         Fixed a format typo in vpause string.
%    02-07-2006      rme         Added check for TheConfig.Estimator.useManualCorrespondences
%                                for backwards config file compatability.


%======================================================================
% INITIALIZATION
%======================================================================
% initialize output arguments
regflag = false;
[p_21_mle_5dof,Cov_21_mle_5dof,isel1,isel2,X_mle] = deal([]);

[scale,J_scale] = magnitude(pp_21(1:3));            % baseline scale [m]
Cov_scale = J_scale * Cov_pp21(1:3,1:3) * J_scale'; % baseline covariance [m^2]
% if we know baseline magnitude with enough precision, use it to set min/max
% scene distance bounds for use during triangulation for outlier rejection.  however,
% if we don't know the scale very precisely, just test if the scene points are in front
% of the camera
MIN_DIST = 0.25*prctile([Z1;Z2],10);% minimum distance to camera 1 [m]
MAX_DIST = 2.0*prctile([Z1;Z2],90); % maximum distance to camera 1 [m]
MAX_DIST = min(MAX_DIST,TheConfig.Data.maxAltitude);
alpha = 1-2*normcdf(-6);            % maximum Mahalanobis distance
MAHAL_THRESH = chi2inv(alpha,5);
if sqrt(Cov_scale) > MAX_DIST;
  scale = 1
  MIN_DIST = 0
  MAX_DIST = inf
  MAHAL_THRESH = inf
end;
USE_NAV_PRIOR_GUESS = false;

% convert pixel coordinates to normalized image coordinates
invK = inv(K);
X1 = invK*homogenize(u1,v1);
X2 = invK*homogenize(u2,v2);
[x1,y1] = dehomogenize(X1);
[x2,y2] = dehomogenize(X2);

%======================================================================
% ROBUST CORRESPONDENCE SET ESTIMATION
%======================================================================
% determine a consistent inlier set from the putative correspondences using
% RANSAC or Least-Median-of-Squares (LMedS)
switch 'lmeds';
case 'ransac'; [eMatrixVec,sel,iter] = estim_E_RANSAC(x1(psel1),y1(psel1),x2(psel2),y2(psel2));
case 'lmeds';  [eMatrixVec,sel,iter] = estim_E_LMedS(x1(psel1),y1(psel1),x2(psel2),y2(psel2),0.60);
end;

if isfield(TheConfig.Estimator,'useManualCorrespondences') && ...
      (TheConfig.Estimator.useManualCorrespondences==true);
  sel = [1:length(psel1)]';
  textcolor('dim','yellow','black');
  fprintf('==>Manual Override\n');
  setTerminal('restore');
end;

% inlier correspondence index
isel1 = psel1(sel);
isel2 = psel2(sel);

% outlier correspondence index
osel1 = psel1; osel1(sel) = [];
osel2 = psel2; osel2(sel) = [];

% number of inliers and outliers found
n_putative = length(psel1);
n_inliers  = length(isel1);
n_outliers = length(osel1);

% print results
textcolor('dim','yellow','black');
fprintf('==>%d Inliers\t%d Outliers\t%.2f%%\t%d Iterations\n', ...
	n_inliers, n_outliers, 100*n_inliers/n_putative, iter);
setTerminal('restore');

% plot correspondence set results
if TheConfig.Plot.inlier_motion;
  private_plot_inlier_motion(I1,I2,u1,v1,u2,v2,isel1,isel2); end;
if TheConfig.Plot.outlier_motion;
  private_plot_outier_motion(I1,I2,u1,v1,u2,v2,osel1,osel2); end;

% check if we have more than the minimim specified points
if fail_min_pts_test(n_inliers,MIN_NUM_PTS); return; end;

%======================================================================
% USING INLIER SET, DETERMINE AN INITIAL SOLUTION FOR RELATIVE POSE
%======================================================================
% normalize pose prior 5 DOF camera measurement
[t_dm,J] = trans2dm(pp_21(1:3));  % direction-magnitude vector & Jacobian w.r.t. translation
npp_21(1:2,1) = t_dm(1:2);        % azimuth & elevation
npp_21(3:5,1) = pp_21(4:6);       % rph Euler angles
J = [J(1:2,:),  zeros(2,3); ...   % 5 DOF measurement Jacobian w.r.t. pp_21
     zeros(3),  eye(3)];
Cov_npp21 = J*Cov_pp21*J';        % 1st order covariance

% now that we have an inlier correspondence set, find a solution for
% relative orientation using our orientation prior as a search constraint
%------------------------------------------------------------------------
[R,t,cost] = relorient_sample(x1(isel1),y1(isel1),x2(isel2),y2(isel2), ...
			      pp_21(4:6),Cov_pp21(4:6,4:6),-1);

% for certain critical camera-center/scene-point configurations (i.e. all
% points lie on a ruled-quadric), we may converge to two ambiguous solutions
% for R,t.  the main idea is to choose one of the solutions based
% upon Mahalanobis distance to our pose prior
%----------------------------------------------------------------------
% for each R,t solution compute its Mahalanobis distance w.r.t. our prior
number_of_solns = size(t,2);
min_mdist = inf;
for ii=1:number_of_solns;
  R_ambig = squeeze(R(:,:,ii));  rph_ambig = rot2rph(R_ambig); t_ambig = t(:,ii);
  % compute normalized 5 DOF measurement unwrapping the solution so that
  % it is consistent with pose prior
  t_dm = trans2dm(t_ambig);
  npp_ambig = [t_dm(1:2); rph_ambig];
  npp_ambig = funwrap([npp_21,npp_ambig],2);
  npp_ambig = npp_ambig(:,2);
  % compute the mahalanobis distance
  mdist(ii) = mahalanobis_dist(npp_ambig,npp_21,Cov_npp21);
  % keep the solution with the minimum Mahalanobis distance
  if mdist(ii) < min_mdist;
    min_mdist = mdist(ii);
    msel = ii;
    rph_o = rph_ambig;
    R_o = rotxyz(rph_o);
    t_o = t_ambig;
  end;
end;

% check mahalanobis distance & print warning
if min_mdist > MAHAL_THRESH;
  setTerminal('warning');
  fprintf('***%s:  Mahalanobis distance %.2f > threshold %.2f...\n',mfilename,min_mdist,MAHAL_THRESH);
  setTerminal('restore');
  %return;
end;

% triangulate to get initial guess for 3D scene points
X_o = triangulate(R_o,t_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);

if USE_NAV_PRIOR_GUESS; % use nav prior as inital guess
  t_o = unitize(pp_21(1:3));
  rph_o = pp_21(4:6);
  R_o = rotxyz(rph_o);
  X_o = triangulate(R_o,t_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);
end;

% check triangulated points using pose prior baseline magnitude to set scale
cpp_21  = [scale*t_o; rph_o];
X_scale = scale*X_o;
sel = enforce_triangulation_constraint(X_scale,MIN_DIST,MAX_DIST,TheConfig);
if TheConfig.Plot.mahal_pose; private_plot_relorient_sample_result(cpp_21,X_scale,sel,scale); end;

% print results
private_print_relorient_sample_result(npp_21,Cov_npp21,R,t,cost,mdist,msel)

% check if we have more than the minimim specified points
if fail_min_pts_test(n_inliers,MIN_NUM_PTS); return; end;

% recalculate our nonlinear optimization starting point using the reduced point set
if length(sel) > 0;
  [R_o,t_o] = relorient_horn(x1(isel1),y1(isel1),x2(isel2),y2(isel2),R_o);
  rph_o = rot2rph(R_o);
  X_o = triangulate(R_o,t_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);
end;

%======================================================================
% REFINE INITIAL GUESS VIA MAXIMUM LIKELIHOOD ESTIMATION
%======================================================================
if USE_NAV_PRIOR_GUESS; % use nav prior as inital guess
  t_o = unitize(pp_21(1:3));
  rph_o = pp_21(4:6);
  R_o = rotxyz(rph_o);
  X_o = triangulate(R_o,t_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);
end;

for k=1:2
  fprintf('\n');
  fprintf('==>%s: Maximum Likelihood Refinement:\n',mfilename);
  % maximum likelhood estimate of relative pose
  MLE_ESTIMATOR = 1; % 1: Rt, 2: Rae, 3: both

  if MLE_ESTIMATOR == 1 || MLE_ESTIMATOR == 3;   % t_hat, rph
    [p_21_mle_6dof,Cov_21_mle_6dof,X_mle,exitflag,output] = ...
	twoview_BundleAdjust_Rt(t_o,rph_o,X_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);

    % map to a nonsingular 5 DOF camera measurement
    [t_dm,Jt] = trans2dm(p_21_mle_6dof(1:3));
    p_21_mle_5dof = [t_dm(1:2); p_21_mle_6dof(4:6)];
  
    % 1st order covariance of 5 DOF camera measurement
    J = [Jt(1:2,1:3), zeros(2,3); ...
	 zeros(3),    eye(3)];
    Cov_21_mle_5dof = J*Cov_21_mle_6dof*J';
  end;

  if MLE_ESTIMATOR == 2 || MLE_ESTIMATOR == 3;   % az,el, rph
    [p_21_mle_5dof,Cov_21_mle_5dof,X_mle,exitflag,output] = ...
	twoview_BundleAdjust_Rae(t_o,rph_o,X_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);

    % map to a singular 6 DOF camera measurement
    [t_mle,Jt] = dm2trans([p_21_mle_5dof(1:2);1]);
    p_21_mle_6dof = [t_mle; p_21_mle_5dof(3:5)];
  
    % 1st order covariance of 6 DOF camera measurement
    J = [Jt(1:3,1:2), zeros(3); ...
	 zeros(3,2),  eye(3)];
    Cov_21_mle_6dof = J*Cov_21_mle_5dof*J';
  end;

  % check triangulated points using pose prior baseline magnitude to set scale
  % look for outliers based upon triangulated points
  sel = enforce_triangulation_constraint(scale*X_mle,MIN_DIST,MAX_DIST,TheConfig);
  if length(sel) > 0;
    textcolor('dim','yellow','black');
    fprintf('==>%s: MLE throwing out %d points based on triangulation constraint.\n', ...
	    mfilename,length(sel));
    setTerminal('restore');
  
    % check if we have more than the minimim specified points
    if fail_min_pts_test(n_inliers,MIN_NUM_PTS); return; end;
    
    % recalculate our nonlinear optimization starting point using the reduced point set
    [R_o,t_o] = relorient_horn(x1(isel1),y1(isel1),x2(isel2),y2(isel2),R_o);
    rph_o = rot2rph(R_o);
    X_o = triangulate(R_o,t_o,u1(isel1),v1(isel1),u2(isel2),v2(isel2),K);
  else;
    break;
  end;
end;


% heuristic: linearly scale the measurement covariance based upon number of inliers.
beta = 3;                                  % scalar defining how much to increase standard deviation
Nmin = TheConfig.ImageFeature.MIN_NUM_PTS; % [Nmin,Nmax] define the abscissa over number of inliers
Ncut = 100;
linearsystem = [Nmin 1; Ncut 1] \ [beta; 1]; % solve for A,b of linear system y = Ax+b
scalefactor =  max(1,linearsystem'*[n_inliers; 1]);
Cov_21_mle_5dof = scalefactor^2*Cov_21_mle_5dof;   % bump up the measurement covariance

% unwrap 5 DOF measurement and check mahalanobis distance
p_21_mle_5dof = funwrap([npp_21,p_21_mle_5dof],2);
p_21_mle_5dof = p_21_mle_5dof(:,2);
mdist_mle1 = mahalanobis_dist(p_21_mle_5dof,npp_21,Cov_npp21);
mdist_mle2 = mahalanobis_dist(npp_21,p_21_mle_5dof,Cov_21_mle_5dof);
[mdist_mle,wrt]  = min([mdist_mle1; mdist_mle2]);
private_print_optimization_results5DOF(p_21_mle_5dof,Cov_21_mle_5dof,exitflag,output,scalefactor,mdist_mle,wrt);
if mdist_mle > MAHAL_THRESH;
  setTerminal('warning');
  fprintf('***%s:  Mahalanobis distance %.2f > threshold %.2f rejecting...\n', ...
	  mfilename,mdist_mle,MAHAL_THRESH);
  setTerminal('restore');
  return;
end;

% check if we should return bundle adjusted 3D structure.
if nargout == 5;  varargout{1} = X_mle; end;
if exitflag > 0;  regflag = true; end;

% plot MLE results
private_plot_pose_results(p_21_mle_6dof,Cov_21_mle_6dof,X_mle,u1(isel1),v1(isel1),scale,I1,TheConfig);

% plot MLE epipolar lines
if TheConfig.Plot.sample_epipolar_lines;
  t_mle   = p_21_mle_6dof(1:3);
  rph_mle = p_21_mle_6dof(4:6);
  R_mle   = rotxyz(rph_mle);
  % project 3D structure to get optimal image points
  P1 = K*[eye(3), zeros(3,1)];
  P2 = K*[R_mle, t_mle];
  [u1_mle,v1_mle] = pinhole_project(P1,X_mle);
  [u2_mle,v2_mle] = pinhole_project(P2,X_mle);
  F_mle = F_from_KRt(R_mle,t_mle,K);
  
  figure(54);
  sample_epipolar_lines(I1,I2,u1_mle,v1_mle,u2_mle,v2_mle,F_mle);
end;


%*********************************************************************************************
function ret = fail_min_pts_test(n_inliers,MIN_NUM_PTS);

ret = false;
% check if we have more than the minimim specified points
if n_inliers < MIN_NUM_PTS;
  % no set of consistent correspondences and model could be found
  setTerminal('warning');
  fprintf('***%s: Putative set %d < MIN_NUM_PTS %d\n', mfilename, n_inliers, MIN_NUM_PTS);
  setTerminal('restore');
  ret = true;
end;


%**********************************************************************************************
function sel = enforce_triangulation_constraint(X_scale,MIN_DIST,MAX_DIST,TheConfig);

if isfield(TheConfig.Estimator,'useManualCorrespondences') && ...
      (TheConfig.Estimator.useManualCorrespondences==true);  
  % do not enforce structure constraint, assume points are a correct match
  sel = [];
else;
  % look for outliers based upon triangulated points
  sel = find((X_scale(3,:) < MIN_DIST) | (X_scale(3,:) > MAX_DIST));
end;
  
if isempty(sel); return; end;

percentBad = length(sel)/size(X_scale,2)*100;
if percentBad > 50;
  msg = sprintf('***%s: Triangulation constraint, rejecting %.2f%%',...
		strrep(mfilename,'_','\_'),percentBad);
  vpause(5,msg,1);
end;

% get correspondence index variables from workspace
isel1 = evalin('caller','isel1');
isel2 = evalin('caller','isel2');
osel1 = evalin('caller','osel1');
osel2 = evalin('caller','osel2');

% reduced correspondence set
tsel1 = isel1(sel);
tsel2 = isel2(sel);
osel1 = [osel1; tsel1];
osel2 = [osel2; tsel2];
isel1(sel) = [];
isel2(sel) = [];
n_inliers  = length(isel1);
n_outliers = length(osel1);
n_putative = n_inliers+n_outliers;

% set correspondence index variable in workspace
assignin('caller','osel1',osel1);
assignin('caller','osel2',osel2);
assignin('caller','isel1',isel1);
assignin('caller','isel2',isel2);
assignin('caller','n_inliers',n_inliers);
assignin('caller','n_outliers',n_outliers);

% print results
textcolor('dim','yellow','black');
fprintf('==>%d Inliers\t%d Outliers\t%.2f%%\tTriangulation constraint\n', ...
	n_inliers, n_outliers, 100*n_inliers/n_putative);
setTerminal('restore');

% plot correspondence set results
if TheConfig.Plot.inlier_motion;
  u1 = evalin('caller','u1');  v1 = evalin('caller','v1');
  u2 = evalin('caller','u2');  v2 = evalin('caller','v2');
  figure(50);
  hold on;
  plot_motion([],u1(tsel1),v1(tsel1),u2(tsel2),v2(tsel2),[],{'Color','r'});
  hold off;
  figure(51);
  hold on;
  plot_motion([],u2(tsel2),v2(tsel2),u1(tsel1),v1(tsel1),[],{'Color','r'});
  hold off;
end;


%*********************************************************************************************
function private_print_relorient_sample_result(npp_21,Cov_npp21,R,t,cost,mdist,msel);
% print pose result
fprintf('\n');
fprintf('==>%s: relorient_sample results:\n',mfilename);
fprintf('        az      el\t\t   rph\t\t   residual   mahalanobius\n');
fprintf(' ---------------------\t------------------------- ---------   ----------\n');
fprintf('nav  %+7.2f  %+7.2f\t%+7.2f  %+7.2f  %+7.2f\n', npp_21*RTOD);
fprintf('+/-  %7.2f  %7.2f  \t%7.2f  %7.2f  %7.2f\n',sqrt(diag(Cov_npp21))*RTOD);
fprintf('\n');

number_of_solns = size(t,2);
for ii=1:number_of_solns;
  R_ambig = squeeze(R(:,:,ii)); t_ambig = t(:,ii);
  % compute normalized 5 DOF measurement unwrapping rph solution so that
  % it is consistent with prior pose
  t_dm = trans2dm(t_ambig);
  rph_ambig = rot2rph(R_ambig);
  rph_ambig = funwrap([npp_21(3:5),rph_ambig],2);
  rph_ambig = rph_ambig(:,2);    
  npp_ambig = [t_dm(1:2); rph_ambig];  
  fprintf('horn %+7.2f  %+7.2f  \t%+7.2f  %+7.2f  %+7.2f  %7.3e', npp_ambig*RTOD, cost(ii));
  if ii == msel;
    % solution chosen based upon Mahalnobius distance
    fprintf('    %7.3fn*\n', mdist(ii));
  else;
    % other ambiguous solution
    fprintf('    %7.3fn\n',mdist(ii));
  end;
end;

%*********************************************************************************************
function private_plot_relorient_sample_result(cpp_21,X_scale,sel,scale);

figure(60); clf; cameratoolbar;
% plot relative pose results using nav scale factor for baseline
plot3(X_scale(1,:),X_scale(2,:),X_scale(3,:),'.','Color',brown);
hold on;
plot3(X_scale(1,sel),X_scale(2,sel),X_scale(3,sel),'r.');
hold off;
titleString = sprintf(['Candidate Image Based Solution, Scale=%.2f\n',...
		       'Camera 1 w.r.t Camera 2'],scale);
title(titleString);
legend('inlier','outlier');
render_cameras(rotxyz(cpp_21(4:6)),cpp_21(1:3),1,0.25,'ggmk');
hold off;
axis equal tight;
view(3); % default 3d view
set(60,'Name','Relorient Horn Result');

%***************************************************************************************
function private_print_optimization_results5DOF(npp_21est,Cov_npp21est,exitflag,output,fudge,mdist,wrt);

if wrt==1; wrt = 'n'; else; wrt = 'c'; end;
sigma = sqrt(diag(Cov_npp21est));
fprintf('     %+7.2f  %+7.2f  \t%+7.2f  %+7.2f  %+7.2f\n', npp_21est*RTOD);
fprintf('+/-  %7.2f  %7.2f  \t%7.2f  %7.2f  %7.2f  (%4.2f*sigma) %7.3f%c\n', ...
	sigma*RTOD,fudge,mdist,wrt);
% optimization unsuccessful
setTerminal('warning');
if exitflag == 0;
  fprintf('Optimization unsuccessful, maximum function evaluations reached.\n');
elseif exitflag < 0;
  fprintf('Optimization unsuccessful, solution did not converge.\n');    
end;
setTerminal('restore');

%***************************************************************************************
function private_print_optimization_results6DOF(npp_21est,Cov_npp21est,exitflag,output);
  
t_est = npp_21est(1:3);
rph_est = npp_21est(4:6);
sigma = sqrt(diag(Cov_p21est));

fprintf('    %+.3f  %+.3f  %+.3f\t%+.3f  %+.3f  %+.3f\n', t_est, rph_est*RTOD);
fprintf('+/- %+.3f  %+.3f  %+.3f\t%+.3f  %+.3f  %+.3f\n', sigma(1:3), sigma(4:6)*RTOD);
% optimization unsuccessful
setTerminal('warning');
if exitflag == 0;
  fprintf('Optimization unsuccessful, maximum function evaluations reached.\n');
elseif exitflag < 0;
  fprintf('Optimization unsuccessful, solution did not converge.\n');    
end;
setTerminal('restore');

%***************************************************************************************
function private_plot_pose_results(p_21est,Cov_p21est,X_est,u,v,scale,Ic,TheConfig);
  
t_est   = scale*p_21est(1:3);
rph_est = p_21est(4:6);
R_est   = rotxyz(rph_est);
X_est   = scale*X_est;

if TheConfig.Plot.mle_pose_scene;
  figure(61); clf; cameratoolbar;
  render_cameras_and_scene(R_est,t_est,X_est,scale,0.25,'ggmk',[],u,v,Ic);
  hold on;
  plot3_multicolor(X_est(1,:),X_est(2,:),X_est(3,:),'.');
  hold off;
  set(61,'Name','Relative Pose and Texture Mapped Scene w.r.t. Camera 1 Frame');
  view(3);
  
end;

if TheConfig.Plot.mle_scene;
  figure(62);
  pos = get(62,'position');
  clf; cameratoolbar; 
  render_scene(X_est(1,:)',X_est(2,:)',X_est(3,:)',[],u,v,Ic);
  hold on;
  plot3_multicolor(X_est(1,:),X_est(2,:),X_est(3,:),'.');
  hold off;
  xlabel('X_{C1}');
  ylabel('Y_{C1}');
  zlabel('Z_{C1}');
  set(62,'Name','Reconstructed Texture Mapped Scene in Camera 1 Frame');
  view(3);
  set(62,'position',pos);
end;

if TheConfig.Plot.mle_pose;
  figure(63); clf; cameratoolbar;
  render_cameras_and_scene(R_est,t_est,X_est,scale,0.25,'ggmk');
  hold on;
  plot3_multicolor(X_est(1,:)',X_est(2,:)',X_est(3,:)','.');
  hold off;
  set(63,'Name','Relative Pose, Scene, and Feature Points w.r.t. Camera 1 Frame');
  view(3);
end;

%***************************************************************************************
function private_plot_inlier_motion(I1,I2,u1,v1,u2,v2,isel1,isel2,osel1,osel2);
n_inliers = length(isel1);
isOutliers = exist('osel1','var') && length(osel1);
% plot inlier motion
figure(50); clf;
hold on;
plot_motion(I1,u1(isel1),v1(isel1),u2(isel2),v2(isel2));
if isOutliers;
  plot_motion([],u1(osel1),v1(osel1),u2(osel2),v2(osel2),[],{'Color','r'});
end;
hold off;
set(50,'Name',sprintf('I1 Inliers Motion Vector: %d',n_inliers));
figure(51); clf;
hold on;
plot_motion(I2,u2(isel2),v2(isel2),u1(isel1),v1(isel1));
if isOutliers;
  plot_motion([],u2(osel2),v2(osel2),u1(osel1),v1(osel1),[],{'Color','r'});  
end;
hold off;
set(51,'Name',sprintf('I2 Inlier Motion Vector: %d',n_inliers));
drawnow;

%***************************************************************************************
function private_plot_outier_motion(I1,I2,u1,v1,u2,v2,osel1,osel2);
% plot outlier motion
n_outliers = length(osel1);
figure(52); clf;
plot_motion(I1,u1(osel1),v1(osel1),u2(osel2),v2(osel2),[],{'Color','r'});
set(52,'Name',sprintf('I1 Outliers Motion Vector: %d',n_outliers));
figure(53); clf;
plot_motion(I2,u2(osel2),v2(osel2),u1(osel1),v1(osel1),[],{'Color','r'});
set(53,'Name',sprintf('I2 Outliers Motion Vector: %d',n_outliers));
