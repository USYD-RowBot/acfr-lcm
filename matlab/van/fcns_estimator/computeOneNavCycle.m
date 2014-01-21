function [sensorEvents,mindex_t] = computeOneNavCycle(nav_t,mindex_t,TheConfig,istrueAugment);
%function [sensorsEvents,mindex_t] = computeOneNavCycle(nav_t,mindex_t,TheConfig,istrueAugment);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-28-2004      rme         Created and written.
%    11-08-2004      rme         Added record_stats
%    12-27-2004      rme         Added path length calculation.
%    01-15-2005      rme         Modified to call bathy_t_struct even when not
%                                using camera observation model.
  
global TheJournal;

% determine when next available asynchronous measurement occurs
% as this determines how forward in time to advance the vehicle state.
[dt,sensorEvents] = checkForAsyncSensorMeas(TheJournal.t,nav_t,mindex_t,TheConfig);

% increment time
TheJournal.t = TheJournal.t + dt;

if istrueAugment; % AUGMENT STATE
  if dt > 0;
    % print to screen
    fprintf('==>%s: Augmenting state vector... ',mfilename);

    % augment state EKF form
    if TheConfig.Estimator.useEkf;
      time = clock;
      augstate_pwc_ekf(dt,TheConfig); %TheJournal
      record_stats('PredictEkf',etime(clock,time),TheJournal.Index.imageQueue,TheConfig);      
    end;
    
    % augment state EIF form
    if TheConfig.Estimator.useEif;
      time = clock;
      augstate_pwc_eif(dt,TheConfig); %TheJournal
      record_stats('PredictEif',etime(clock,time),TheJournal.Index.imageQueue,TheConfig);
    end;
    
    % update our book keeping indices
    augstate_bookkeep; %TheJournal

    % display the augmented state normalized correlation matrix
    if TheConfig.Plot.plot_corrcoeff;
      figure(10); clf;
      plotSigmaLambda(TheJournal,TheConfig);
      set(10,'Name','Normalized Covariance & Information Matrix');
    end;
    
    % print to screen
    fprintf('Nf:%d  Naug:%d\n',TheJournal.Index.Nf,TheJournal.Index.Naug);    
    
  else; % dt <= 0
    setTerminal('error');
    fprintf('***%s: Error augmentation dt<=0, dropping to keyboard...\n',...
	    mfilename,dt,sensorEvents{:});
    setTerminal('restore');
    keyboard;
  end;
  
else; % PREDICTION
  if dt > 0;
    if TheConfig.Estimator.useEkf;
      % time propogate EKF vehicle state forward via vehicle dynamics
      time = clock;      
      predict_pwc_ekf(dt,TheConfig); % TheJournal
      record_stats('PredictEkf',etime(clock,time),-1,TheConfig);
    end;
  
    if TheConfig.Estimator.useEif;
      % time propogate EIF vehicle state forward via vehicle dynamics
      time = clock;
      predict_pwc_eif(dt,TheConfig); % TheJournal
      record_stats('PredictEif',etime(clock,time),-1,TheConfig);
    end;
    
  else; % dt <= 0
    setTerminal('warning');
    fprintf('***%s: Warning prediction dt = %g:',mfilename,dt);
    fprintf(' %s',sensorEvents{:});
    fprintf('\n');
    setTerminal('restore');
  end;
end;

% state measurement update
%=================================================================
% add current sensor measurements to meas_t which is an array of structures
[meas_t,mindex_t] = poll_nav_sensors(sensorEvents,nav_t,mindex_t,TheConfig);
% update vehicle state
process_measurements(meas_t,TheConfig); % TheJournal

% update pathlength
pathlength(TheConfig); % TheJournal

% put the RDI beam range measurements into the local-level coordinate frame
% based upon filtered vehicle position
if any(strcmp('RDI',sensorEvents));
  bathy_t_struct('push',nav_t,mindex_t,TheConfig);
end;
