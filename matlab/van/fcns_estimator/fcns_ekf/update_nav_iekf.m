function [] = update_nav_iekf(meas_t,TheConfig)
%function [] = update_nav_iekf(meas_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-19-2004      rme         Created from ekfupdatenav.m
%    03-29-2004      rme         Removed LINEAR_MEAS argument
%    03-30-2004      rme         Added z_raw argument so that raw
%                                attitude measurements can be properly
%                                wrapped before using in EKF update.
%    04-12-2004      rme         Updated to pass index_t structure to OMfhandle
%    05-10-2004      rme         Added innovation to output args
%    09-09-2004      rme         Renamed from iekfupdatenav.m to update_nav_iekf.m
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Modified to work with preallocated TheJournal
%    11-26-2004      rme         Modified to loop over meas_t and lump into a single measurement.

global TheJournal;

% shorthand index into augmented state vector
Xv_i = TheJournal.Index.Xv_i;  % vehicle state index
Xf_i = TheJournal.Index.Xf_i;  % feature state index
Xa_i = TheJournal.Index.Xa_i;  % "all" state index

% pointers into TheJournal
mu_0    = TheJournal.Ekf.mu(Xa_i);
Sigma_0 = TheJournal.Ekf.Sigma(Xa_i,Xa_i);

% setup iterated EKF loop
maxIterations = 10;
mu_i = mu_0;
[Hv_i,K_i,nu_i,S_i] = deal([]);
error_i = inf;
error_stat = zeros(maxIterations,1);
for ii=1:maxIterations;
  Hv_iprev = Hv_i; % store Jacobian from previous iteration
  
  % loop over meas_t and aggregate measurements
  %-----------------------------------------------
  [z_predict_i,z_fix,R_fix,Hv_i] = deal(cell(size(meas_t)));
  for jj=1:length(meas_t);
    % evaluate the predicted measurement and the Jacobian of the observation
    % model.  allow passing of optional arguments to observation function.
    % note that the observation function takes the measurement "z" as an
    % argument and also returns it as an output.  in most cases the output z
    % argument will remain unchanged from the input.  however this
    % input/output capability facilitates angular unwrapping of raw attitude
    % measurements to be consistent with the predicted unwrapped state estimate.
    %----------------------------------------------------------------------------
    % measurement prediction
    [z_predict_i{jj},z_fix{jj},R_fix{jj},Hv_i{jj}] = ...
	feval(meas_t(jj).omfhandle, mu_i(Xv_i), TheJournal.Index, ...
	      meas_t(jj).z, meas_t(jj).R, meas_t(jj).varargin{:});      
    
    if isempty(Hv_i{jj});
      % numerically evaluate the Jacobian of the observation model w.r.t. vehicle state vector
      fprintf('==>%s: Computing numerical Jacobian Hv_i for %s\n',mfilename,meas_t(jj).omfhandleString);
      Hv_i{jj} = numerical_jacobian(meas_t(jj).omfhandle, mu_i(Xv_i), z_predict_i{jj}, [], ...
				    TheJournal.Index, meas_t(jj).z, meas_t(jj).R, meas_t(jj).varargin{:});
    end;
  end; % for jj=1:length(meas_t)
  
  % lump into a single large measurement vector
  %-----------------------------------------------
  z_predict_i = vertcat(z_predict_i{:});
  z_fix       = vertcat(z_fix{:});
  R_fix       = sparse(blkdiag(R_fix{:}));
  Hv_i        = sparse(vertcat(Hv_i{:}));
  
  % iterative innovation and covariance
  %--------------------------------------------
  nu_iprev = nu_i; % store innovation from previous iteration
  S_iprev  = S_i;  % store innovation covariance from previous iteration
  nu_i = z_fix - z_predict_i - Hv_i*(mu_0(Xv_i) - mu_i(Xv_i));
  S_i  = R_fix + spdproduct(Sigma_0(Xv_i,Xv_i),Hv_i');
  
  % iterative Kalman gain
  %--------------------------------------------
  K_iprev = K_i; % store gain matrix from previous iteration
  K_i = Sigma_0(:,Xv_i)*Hv_i'*spdinverse(S_i);

  % iterative state update
  %--------------------------------------------
  mu_iprev = mu_i; % store updated state from previous iteration
  mu_i = mu_0 + K_i*nu_i;

  % check error magnitude condition
  %---------------------------------------
  TOL = 1e-12; % (10^-6)^2
  error_iprev = error_i;
  evec = (mu_i - mu_iprev);
  error_i = evec'*evec;
  error_stat(ii) = error_i;
  if error_i < TOL;
    break; % exit loop
  elseif error_i > error_iprev;
    % increase in error, terminate with previous iteration estimate
    mu_i   = mu_iprev;
    Hv_i   = Hv_iprev;
    K_i    = K_iprev;
    nu_i   = nu_iprev;
    S_i    = S_iprev;
    break; % exit loop
  end;
end; % for ii=1:maxIterations

if ii==maxIterations;
  for ii=1:maxIterations;
    fprintf('==>%s: IEKF i=%d  error=%e  ', mfilename, ii, error_stat(ii));
    for jj=1:length(meas_t);
      fprintf('%s ', meas_t(jj).omfhandleString);
    end;
    fprintf('\n');
  end;
end;

% store the updated mean
TheJournal.Ekf.mu(Xa_i) = mu_i;

% covariance update (based on Joseph form)
%--------------------------------------------
% the expressions (1), (2), and (3) are equivalent. HOWEVER (2) and (3) seem
% to be better numerically conditioned (keep covariance matrix symmetry)
% while (3) seems to be the fastest compution (according to matlab function
% profiling.)

% NOTE: experimentally, i found that it was faster to perform the covariance
% calculation useing regular matrices.  the overhead associated with sparse
% matrices Hv and K increased the compution time.

% expression 1
TheJournal.Ekf.Sigma(Xa_i,Xa_i) = Sigma_0 - 2*K_i*Hv_i*Sigma_0(Xv_i,:) + K_i*S_i*K_i';

% expression 2
%PHK = Paug_0(:,Xvi)*Hv'*K';
%Paug_i = Paug_0 - PHK - PHK' + K_i*S_i*K_i';

% expression 3
%persistent Iaug;
%if index_t.Naug ~= size(Iaug,1)
%  Iaug = speye(index_t.Naug);
%end
%IminusKH = (Iaug-Kaug*Haug);
%Paug_i = (IminusKH)*Paug_0*(IminusKH)' + Kaug*R*Kaug';

% preserve symmetry
TheJournal.Ekf.Sigma(Xa_i,Xa_i) = (TheJournal.Ekf.Sigma(Xa_i,Xa_i)+TheJournal.Ekf.Sigma(Xa_i,Xa_i)')/2;
