% HISTORY      WHO     WHAT
%----------    ----    -------------------------------------
% 03-19-2004   rme     Added IEFK update.
% 03-31-2004   rme     added bathy_t_struct fcn call to TheJournal.
% 04-08-2004   rme     modified to use mindex_t
% 04-09-2004   rme     added checkForAsyncSensorMeas function call and
%                      completely reorganized code.
% 04-14-2004   rme     Added plot_ssa_xy code
% 05-10-2004   rme     Record innovation for post-analysis
% 07-01-2004   rme     Modified to use ekfpredict_pwc
% 09-08-2004   rme     Inserted eifpredict_pwc
% 10-14-2004   rme     Added camera event and augment event print statements
% 10-18-2004   rme     Added config EKF & EIF checks, partial state recovery, pcg
% 10-21-2004   rme     Added assumed density filtering (ADF) EIF code
% 10-28-2004   rme     Major code reorganization.
% 11-04-2004   rme     Consolidated graphics code.
% 11-11-2004   rme     Added vpause.dat
% 12-06-2004   rme     Updated arguments to plotTraj.m
% 01-14-2005   rme     Updated to take snapshots of TheJournal when not using the
%                      camera but still doing delayed-state processing
% 01-19-2005   rme     Moved EIF pre-camera reg state recovery to processLoop.m
%                      Moved stuffing of EIF covariance bounds to processLoop.m
% 01-10-2006   rme     Moved placement of exit status check within the loop
%                      so that it properly terminates when not doing delayed-state
%                      filtering.
% 04-26-2006   rme     Moved vpause.dat to /tmp
% 05-02-2006   rme     Hacked in done=true if DVL stream empty
tick = 0;
done = false;
while ~done;
  %===================================================
  % PROPOGATE STATE VECTOR FORWARD IN TIME
  %===================================================
  [sensorEvents,mindex_t] = computeOneNavCycle(nav_t,mindex_t,TheConfig,0);

  % take a snapshot of the filtered vehicle state
  if TheConfig.Estimator.record_ssv && TheConfig.Estimator.useEkf;
    cc = max(1,ssv_t(end).cc-1);
    ssvEvent = (TheJournal.t - ssv_t(end).t(cc)) > TheConfig.Estimator.ssvSamplePeriod;
    if ssvEvent; snapshot_ssv(-1,TheConfig); end;
  end;
  
  % check if DVL data is spent
  rdiEmpty = (mindex_t.RDI == length(nav_t.RDI.rovtime));
  if rdiEmpty;
    snapshot_ssv(9999,TheConfig) % hack so that this ssv_t gets logged to disk
    done = true; 
  end;
  
  
  %========================================================
  % CHECK IF A CAMERA EVENT OCCURED DURING NAV CYCLE
  %========================================================
  cameraEvent = any(strcmp(sensorEvents,'PXF')); % true if camera event occured
  if ~cameraEvent; continue;  end; % reloop back to start
  
  % increment the camera measurement index
  imageNumber  = nav_t.PXF.imgnum(mindex_t.PXF);
  mindexCopy_t = mindex_t;
  mindex_t     = incrementPxfMeasurementIndex(nav_t,mindex_t,TheConfig);
  fprintf('\n');
  fprintf('=========================================================================\n');  
  textcolor('bright','cyan','black');
  fprintf('==>%s: Camera event occured: %d\n',mfilename,imageNumber);
  setTerminal('restore');

  % add the camera event to the process queue
  TheJournal.Index.imageQueue = imageNumber;
  
  % check exit condition
  if nav_t.PXF.imgnum(mindexCopy_t.PXF) == TheConfig.Data.imageSequence(end);
    done = true; % set exit flag
  end;
  
  % take a snapshot of the filtered vehicle state
  if TheConfig.Estimator.record_ssv && TheConfig.Estimator.useEkf;
    if TheConfig.Plot.plot_ssv; 
      figure(1); plot_ssv(nav_t,ssv_t);
    end;
    snapshot_ssv(imageNumber,TheConfig);
  end;
  
  %========================================================
  % DISPLAY SOME GRAPHICS
  %========================================================
  % plot innovation
  if TheConfig.Plot.plot_nu_t && TheConfig.Estimator.record_innov;
    figure(420);
    plot_nu_t(nu_t);
  end;
  
  % plot trajectory
  if TheConfig.Plot.plot_links_xy;
    if ~exist('lastNum','var');
      lastNum = TheConfig.Data.imageSequence(1)+1;
    end;
    figure(5);
    plotTraj(2,TheJournal,nav_t,[lastNum,mindex_t.PXF],[],TheConfig);
    set(5,'Name','Estimated XY Topology');
    lastNum = mindexCopy_t.PXF;
    drawnow;
  end;

  % plot computation time
  if TheConfig.Estimator.record_stats && TheConfig.Plot.plot_stats;
    figure(11);
    plot_stats(stats_t);
    set(11,'Name','Process Time');
  end
  
  %========================================================
  % CHECK IF WE SHOULD DO DELAYED STATE PROCESSING
  %========================================================
  if ~TheConfig.Estimator.useDelayedStates; continue; end; % reloop back to start
  
  % augment our state vector
  [sensorEvents,mindex_t] = computeOneNavCycle(nav_t,mindex_t,TheConfig,1);
  
  
  %========================================================
  % PROCESS CAMERA MEASUREMENTS
  %========================================================
  % recover our predicted robot-covariance & state
  %---------------------------------------------------------------
  if TheConfig.Estimator.useEif;
    if TheConfig.Estimator.useFullStateRecovery;
      aclock('tic');
      recover_state_full(3); %TheJournal
      record_stats('EifDA1',aclock('toc'),TheJournal.Index.imageQueue,TheConfig);
    else;
      aclock('tic');
      recover_state_pcg(TheConfig.Estimator.pcgMaxIter,TheConfig.Estimator.groupSize,3); %TheJournal
      record_stats('EifDA1',aclock('toc'),TheJournal.Index.imageQueue,TheConfig);    
    end;
  end;

  % generate RDI measured bathy for image
  %---------------------------------------------------------------	
  TheBathy(TheJournal.Index.fn,1) = bathy_t_struct('pop',TheConfig);
  
  useCamera = ~isempty(TheConfig.ObservationModel.PXF);
  if useCamera;
    % take a snapshot *before* image reg
    %---------------------------------------------------------------	
    snapshot_TheJournal(mindex_t,TheConfig,imageNumber,'before');
    % go go gadget "image registration"
    %---------------------------------------------------------------	
    computeOneCamCycle(nav_t,mindexCopy_t,ssv_t,TheConfig);    
  end;
  
  % stuff the covariance estimate for the current image into TheJournal
  fn   = TheJournal.Index.fn;
  Xp_i = TheJournal.Index.Xp_i;
  Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i);
  TheJournal.Eif.SigmaBound{fn}  = TheJournal.Eif.SigmaCol(Xf_i,:);
  TheJournal.Eif.SigmaBound0{fn} = TheJournal.Eif.SigmaCol(Xf_i,:); % store initial bound for post-analysis
  
    
  %========================================================  
  % TAKE A SNAPSHOT OF OUR STATE AND WRITE TO DISK
  %========================================================
  snapshot_TheJournal(mindex_t,TheConfig,imageNumber,'after');
  snapshot_bathy(TheConfig);
  
  % interactive visual pause
  if exist([tempdir,'vpause.dat'],'file'); 
    TheConfig.Data.vpause = load([tempdir,'vpause.dat']); 
  end;
  vpause(TheConfig.Data.vpause,[],0);
  
end; % while ~done

% plot trajectory
if TheConfig.Plot.plot_links_xy;
  if ~exist('lastNum','var');
    lastNum = TheConfig.Data.imageSequence(1)+1;
  end;
  figure(5);
  plotTraj(2,TheJournal,nav_t,[lastNum,mindexCopy_t.PXF],[],TheConfig);
  lastNum = mindexCopy_t.PXF;
  drawnow;
end;


%========================================================
% CLEAN UP
%========================================================
setTerminal('normal');
fprintf('\n');
fprintf('==>%s: VAN finished ...\n',mfilename);

% archive snapshot data
%--------------------------------------------------------
archive_records(TheConfig);

