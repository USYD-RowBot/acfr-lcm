% process_init.m initializes book keeping
% HISTORY      WHO     WHAT
%----------    ----    -------------------------------------
% 04-05-2004   rme     Initalized time TheJournal.t to earliest sensor meas
% 04-08-2004   rme     Updated to use mindex_t structure
% 04-10-2004   rme     Completely reorganized the calculation of initial
%                      vehicle state to use TAU seconds worth of nav data prior
%                      to 1st camera measurement to get state and
%                      covariance at time t0.
% 10-18-2004   rme     Added information form and Rchol to TheJournal
% 10-19-2004   rme     Added initialization of TheJournal.Eif.LambdaADF & TheJournal.Ekf.SigmaADF
% 11-03-2004   rme     removed snapshot of ssa_t & ssv_t
% 11-11-2004   rme     Added vpause.dat
% 12-20-2004   rme     Moved Bathy structure out from under the TheJournal
%                      and into a new global variable TheBathy
% 01-14-2004   rme     Fixed a bug with setTerminal('emphasis') call.
% 03-23-2005   rme     Print message about saving config.
% 01-11-2006   rme     Moved saving of TheConfig until after it is updated
%                      by initializeTheJournal().
% 04-26-2006   rme     Moved vpause.dat to /tmp


% declare globals
global TheJournal TheBathy ssv_t ssa_t stats_t;

%=========================================================================
% remove temporary processing files associated with previous run
%=========================================================================
tmp = [TheConfig.Data.outdir,'proc/'];
if exist(tmp,'dir');
  setTerminal('emphasis');
  ret = lower(input(sprintf('Wipe out %s? [n] ',tmp),'s'));
  setTerminal('restore');
  if strncmp(ret,'y',1);
    fprintf('erasing %s... ',tmp);
    rmdir([TheConfig.Data.outdir,'proc/'],'s'); % recursively remove
    mkdir(TheConfig.Data.outdir,'proc');
    fprintf('done\n');
  else;
    fprintf('not erased, files will be overwritten\n');
  end;
else;
  mkdir(TheConfig.Data.outdir,'proc');  
end;

%============================================================================
% initialize TheJournal and return an updated TheConfig
%============================================================================
TheConfig = initializeTheJournal(TheConfig); %TheJournal
TheBathy  = initializeTheBathy([]);

% save a copy of our config file
fprintf('saving a copy of our config... ');
save([tmp,'TheConfig'],'TheConfig');
fprintf('done\n');

%==============================================================================
% initally assign vehicle state vector to have large uncertainties and
% process a few seconds worth of nav to get updated estimates of vehicle
% states and covariance.  take this updated state estimate as our initial state.
%==============================================================================
% setup measurement index structure associated with the 1st image in the sequence
[mindex_t,TheJournal.t] = initializeMeasurementIndex(nav_t,TheConfig,-10);

% main idea is to process a few seconds worth of nav up to the 1st camera meas
% to get a good initalization for the vehicle state elements
mindex_t = initialize_vehicle_state(nav_t,mindex_t,TheConfig); %TheJournal

% check if we should add the camera to vehicle transform to the vehicle
% state vector as an unknown parameter to estimate.
%------------------------------------------------------------------
if ~isempty(TheConfig.ObservationModel.PXF) && TheConfig.Estimator.estimateCameraXform == true;
  % store the transform x_vc as a "feature" state
  TheJournal = augment_state(TheJournal,TheJournal.Index.Xpi,[],-1);
  % grab the feature index and store it
  x_vc_i = TheJournal.Index.Xf{TheJournal.Index.fn};
  TheJournal.Index.x_vc_i = x_vc_i;
  % replace the delayed vehicle state with the transform parameters
  TheJournal.Ekf.mu(x_vc_i) = TheConfig.SensorXform.PXF.x_vs;
  TheJournal.Ekf.Sigma(x_vc_i,:) = 0;
  TheJournal.Ekf.Sigma(:,x_vc_i) = 0;
  TheJournal.Ekf.Sigma(x_vc_i,x_vc_i) = TheConfig.SensorXform.PXF.P_vs;
end;

%==================================================================
% snapshot state
%==================================================================
if TheConfig.Estimator.record_ssv && TheConfig.Estimator.useEkf;
  snapshot_ssv(-1,TheConfig);
end;

% setup vpause.dat
system(sprintf('echo %g > %s',TheConfig.Data.vpause,[tempdir,'vpause.dat']));

% clean up any temporary variables
keep TheConfig TheJournal TheBathy mindex_t nav_t ssa_t ssv_t stats_t;

fprintf('==>%s: ',mfilename);
if TheConfig.Estimator.inferenceEif;
  fprintf('inference='); setTerminal('emphasis'); fprintf('Eif\t'); setTerminal('restore');
  if TheConfig.Estimator.useSeifsDA;
    fprintf('data_association='); setTerminal('emphasis'); fprintf('Seifs\t'); setTerminal('restore');
  else;
    fprintf('data_association='); setTerminal('emphasis'); fprintf('CI\t'); setTerminal('restore');
  end;
else;
  fprintf('inference='); setTerminal('emphasis'); fprintf('Ekf\t'); setTerminal('restore');
end;
setTerminal('restore');
fprintf('\n');
fprintf('\n');
