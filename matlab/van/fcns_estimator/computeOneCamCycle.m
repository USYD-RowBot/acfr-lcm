function computeOneCamCycle(nav_t,mindex_t,ssv_t,TheConfig)
%function computeOneCamCycle(nav_t,mindex_t,ssv_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created and written.
%    04-12-2004      rme         Modified vpause string.
%    04-14-2004      rme         Changed to augment state vector
%                                with a copy of vehicle pose *after* the 
%                                camera update is done. This avoids a
%                                singular Paug covariance matrix during
%                                the update.
%    05-10-2004      rme         Record innovation for post-analysis
%    09-09-2004      rme         Deleted obsolete portions of code.
%                                Changed looping over fni to process best candidates first
%    10-28-2004      rme         Major code reorganization. 
%    11-04-2004      rme         Renamed to computeOneCamCycle.m
%    11-11-2004      rme         Added TheConfig.Estimator.inferenceEif
%    11-11-2004                  Changed link_hypothesis_ekf.m to link_hpothesis.m
%    11-12-2004      rme         Added wrapper call to extract_poses.m
%    11-14-2004      rme         Modified bathy_t_struct.m syntax
%    11-26-2004      rme         Modified to process all camera measurements in one
%                                lumped EKF/EIF update.
%    11-27-2004      rme         Added PCG state recovery prior to making camera measurement.
%    11-28-2004      rme         Added CI covariance bound update
%    12-20-2004      rme         Moved Bathy structure out from under the TheJournal
%                                and into a new global variable TheBathy
%    12-21-2004      rme         Added altratio as output argument to linkgen_hypothesis.m
%    01-14-2005      rme         Moved state recovery and TheBathy to processLoop.m
%    01-19-2005      rme         Got rid of CI update of covariance bounds and switched
%                                to Kalman update using marginalized pdf
%    01-22-2006      rme         Added check for manually registered images in link hypothesis.
%    02-07-2006      rme         Added check for TheConfig.Estimator.useManualCorrespondences
%                                for backwards config file compatability.

global TheJournal;

fprintf('==>%s: Processing camera event...\n',mfilename);

% hypothesize potential overlapping image pairs
%--------------------------------------------------
% fni, fnj are feature numbers of candidate overlapping image pairs
[fni,fnj,probability,mu_d,sigma_dd,altratio] = link_hypothesis(TheConfig); %TheJournal

% check for any manually registered pairs and select them
if isfield(TheConfig.Estimator,'useManualCorrespondences') && ...
      (TheConfig.Estimator.useManualCorrespondences==true);
  inumj = TheJournal.Index.featureLUT(end-1);
  inumi = TheJournal.Index.featureLUT(fni);
  fname = sprintf('%dmanual/Manual-%04d-*.dat',TheConfig.Data.outdir,inumj);
  dir_t = dir(fname);
  for ii=1:length(dir_t);
    inumk = strread(dir_t(ii).name,'Manual-%*d-%d.dat');
    if ~any(ismember(inumi,inumk)); % inumk not in the plink list
      fnk = find(TheJournal.Index.featureLUT == inumk);
      if ~isempty(fnk);
	fni = [fni; fnk];    % add the manual pair to the proposed link list
	fnj = [fnj; fnj(1)];
	mu_d = [mu_d; -1];
	sigma_dd = [sigma_dd; -1];
	altratio = [altratio; -1];
      end;
    end;
  end;
end;

% try to register the proposed image pairs
%--------------------------------------------------
nMeas = 0;
nPairs = length(fni);
for k=1:nPairs;
  % print remaining candidate pairs
  private_print_pairs(nPairs,k,probability,mu_d,sigma_dd,altratio,fni,fnj);

  % extract pose information from our state estimate
  [x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni(k),fnj(k),TheConfig);
  
  % this function does the two-view registration
  [tmpMeas_t,regflag] = add_camera_sensor_meas(nav_t,fni(k),fnj(k),x_lvi,x_lvj,x_vc,Sigma,TheConfig);
  if regflag == true;
    regString = '==>REGISTERED';
    meas_t(nMeas+1) = tmpMeas_t;
    nMeas = nMeas+1;
  else;
    regString = '***FAILED';
  end;
	  
  % allow user to pause processing
  %--------------------------------------------------
  buffString = sprintf('Images:(%4d,%4d) Features:(%3d,%3d)', TheJournal.Index.featureLUT(fni(k)), ...
		       TheJournal.Index.featureLUT(fnj(k)), fni(k),fnj(k));
  vpause(TheConfig.Data.vpause,[regString,' ',buffString]);
  
end; % for k=1:length(fni)

% update our data association covariance bounds
%---------------------------------------------------------------
if TheConfig.Estimator.useEif && nMeas > 0;
    
  % update bounds with Kalman filter using marginalized pdf
  for ii=1:length(meas_t);
    updateCovBound(meas_t(ii),TheConfig);
  end;

  %## note: used to recover the robot state and covariance for the current image
  %## and then update the covariance bounds for other images using CI.  *however*, i 
  %## recently realized that the raw relative pose measurement z_ji is correlated with 
  %## the *updated* robot state.  this makes calculating the pseudo pose measurement covariance
  %## non-trivial.  therefore, to avoid this issue i update the data association covariance
  %## bounds using the raw sensor measurement and *predicted* robot state which are
  %## uncorrelated.  only *after* computing the new covariance bounds do i recover the
  %## updated robot state and covariance.
  % Covariance Intersection update for other states
  %deltaT = 0;
  %aclock('tic');
  %if TheConfig.Plot.CIfusion; 
  %  figure(13); clf; set(13,'Name','CI Fusion');
  %end;
  %deltaT = aclock('toc');
  %for ii=1:length(meas_t);
  %  if TheConfig.Plot.CIfusion; subplot(1,length(meas_t),ii); end;
  %  % use Covariance Intersection to update the Sigma bounds
  %  [P_x_lvi_CI,omega,dt] = CIfusion(TheJournal,meas_t(ii),TheConfig);
  %  % stuff updated covariance bound into TheJournal
  %  TheJournal.Eif.SigmaCI{meas_t(ii).fni} = P_x_lvi_CI;
  %  deltaT = deltaT+dt;
  %end; % for ii=1:length(meas_t)
  %record_stats('EifDA2',deltaT,TheJournal.Index.imageQueue,TheConfig);
end; %if TheConfig.Estimator.useEif


% process camera measurements
%-----------------------------------------------------------
if nMeas > 0;
  % note that process_measurements also does state recovery for EIF internally
  process_measurements(meas_t,TheConfig); % TheJournal

  % if the camera xform is included as a random parameter in our state vector, then
  % print its updated estimate to the screen
  if TheConfig.Estimator.estimateCameraXform;
    x_vc_i = TheJournal.Index.x_vc_i;
    fprintf('Improved x_vc estimate:\n');
    print_pose_prior(TheJournal.Ekf.mu(x_vc_i),TheJournal.Ekf.Sigma(x_vc_i,x_vc_i),'_vc',1,1);
  end;
end; % if nMeas > 0


%======================================================================================
function private_print_pairs(nPairs,nCurr,probability,mu_d,sigma_dd,altratio,fni,fnj);
nRemaining = nPairs-nCurr+1;
textcolor('reset','magenta','black');
fprintf('\n');
fprintf('==>%s: %d remaining testlinks\n',mfilename,nRemaining);
fprintf('\tprob\tratio\tmu_d\tstd\t(fni, fnj)\n');
for ii=1:length(fni)
  fprintf('\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t(%d, %d)',...
	  probability(ii), altratio(ii), mu_d(ii), sigma_dd(ii), fni(ii), fnj(ii));
  if ii==nCurr; fprintf('*'); end; % highlight current pair with asterisk
  fprintf('\n');
end;
setTerminal('restore');
