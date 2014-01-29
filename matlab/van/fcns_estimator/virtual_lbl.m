function virtual_lbl(TheConfig);
  
global TheJournal;

fn   = 24;
%fn = 769; % 1505
Xp_i = TheJournal.Index.Xp_i;
Xv_i = TheJournal.Index.Xv_i;
Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i);

x_lf = TheJournal.Eif.mu(Xf_i);
%x_lf = TheJournal.Eif.mu(Xf_i) + 1.5*[0.83;0.55;zeros(4,1)];

% stuff the measurement and covariance
meas_t.omfhandle = @om_lbl_xyz;
meas_t.omfhandleString = func2str(meas_t.omfhandle);
meas_t.isaDelayedStateMeas  = false;
meas_t.z        = x_lf(1:3);
meas_t.R        = eye(3);
meas_t.varargin = {zeros(6,1)}; % sensor pose w.r.t. to vehicle frame

% update vehicle state
process_measurements(meas_t,TheConfig); % TheJournal

% update our covariance estimate associated with the current image
if TheConfig.Estimator.useFullStateRecovery;
  recover_state_full(3); %TheJournal
else;
  recover_state_pcg(TheConfig.Estimator.pcgMaxIter,TheConfig.Estimator.groupSize,3); %TheJournal
end;

