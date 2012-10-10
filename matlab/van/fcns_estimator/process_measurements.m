function [] = process_measurements(meas_t,TheConfig)
%function [] = process_measurements(meas_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-09-2004      rme         Created and written.
%    04-12-2004      rme         Updated to pass TheJournal.Index argument 
%                                to observation model
%    05-10-2004      rme         Record innovation for post-analysis
%    09-08-2004      rme         Added eifupdatenav.m
%    10-17-2004      rme         Added TheConfig EKF & EIF checks and partial state recovery
%    10-18-2004      rme         Added EKF_ONLY flag to handle state initialization 
%                                during process_init.m
%    10-21-2004      rme         Added assumed density filtering (ADF) EIF code
%    10-28-2004      rme         Major code reorganization.
%    11-08-2004      rme         Added record_stats
%    11-27-2004      rme         Relax state after Eif aug update using SLR.

global TheJournal;

if isempty(meas_t); return; end;

% segregate measurements into either navigation or augmented state measurements by
% keying off delayed state measurement flag
[iids,iinav] = deal([]); % delayed state & nav measurement index into meas_t
for ii=1:length(meas_t);
  if meas_t(ii).isaDelayedStateMeas;
    % measurement involves delayed states (e.g. camera measurement)
    iids = [iids,ii];
  else;
    iinav = [iinav,ii];
  end;
end;

%===============================================
% EKF UPDATE
%===============================================
if TheConfig.Estimator.useEkf;
  if ~isempty(iinav); % process nav measurements (only involves vehicle state)
    % update EKF state
    aclock('tic');
    update_nav_iekf(meas_t(iinav),TheConfig); %TheJournal
    record_stats('UpdateEkf',aclock('toc'),-1,TheConfig);
    %if RECORD_NU;   % record innovation for post-processing analysis
    %nu_t = record_innovation(nu_t,meas_t(iinav).OMhandle,TheJournal.t,nu,S);
    %end;
  end; % if ~isempty(iinav);
  
  if ~isempty(iids); % process delayed state measurements
    % update EKF state
    aclock('tic');
    update_aug_iekf(meas_t(iids),TheConfig); %TheJournal
    record_stats('UpdateEkf',aclock('toc'),TheJournal.Index.imageQueue,TheConfig);
  end; % if ~isempty(iids);
  
end; % if TheConfig.Estimator.useEkf

%===============================================
% EIF UPDATE
%===============================================
if TheConfig.Estimator.useEif;
  if ~isempty(iinav); % process nav measurements (only involves vehicle state)
    % update EIF state
    aclock('tic');
    update_nav_eif(meas_t(iinav),TheConfig); %TheJournal
    % recover current robot mean
    recover_state_partial(TheJournal.Index.Xv_i); % TheJournal
    record_stats('UpdateEif',aclock('toc'),-1,TheConfig);
  end; % if ~isempty(iinav)

  if ~isempty(iids); % process delayed state measurements
    % update EIF state
    b1 = aclock('tic');
    update_aug_eif(meas_t(iids),TheConfig); %TheJournal
    % relax map
    if TheConfig.Estimator.useFullStateRecovery;
      recover_state_full(3);
    else;
      % use preconditioned conjugate gradient to recover an estimate of state mean
      recover_state_pcg(TheConfig.Estimator.pcgMaxIter,TheConfig.Estimator.groupSize,1); %TheJournal
    end;
    record_stats('UpdateEif',aclock('toc'),TheJournal.Index.imageQueue,TheConfig);
  end; % if ~isempty(iids)
  
end; % if TheConfig.Estimator.useEif
  

%===========================================================================
function private_printaug_ekf(meas_t,string)

global TheJournal;
  
% index pointers
fni   = meas_t(ii).fni;   % feature numbers
fnj   = meas_t(ii).fnj;
Xfi_i = meas_t(ii).Xfi_i; % feature indexes
Xfj_i = meas_t(ii).Xfj_i;

% print "before" info to screen
fprintf('%s EKF Augmented State Measurement: (%d,%d)\n',string,fni,fnj);
print_pose_prior(TheJournal.Ekf.mu(Xfi_i),TheJournal.Ekf.Sigma(Xfi_i,Xfi_i),num2str(fni),1,0);
print_pose_prior(TheJournal.Ekf.mu(Xfj_i),TheJournal.Ekf.Sigma(Xfj_i,Xfj_i),num2str(fnj),0,1);
  
