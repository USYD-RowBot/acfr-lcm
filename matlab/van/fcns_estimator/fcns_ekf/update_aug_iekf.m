function [] = update_aug_iekf(meas_t,TheConfig)
%function [] = update_aug_iekf(meas_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-19-2004      rme         Created from ekfupdateaug.m
%    04-06-2004      rme         Removed pinv() during Kaug_i calc
%    04-12-2004      rme         Updated argument list to use Xaug,Paug
%                                instead of TheJournal directly
%    05-10-2004      rme         Added innovation to output args
%    09-09-2004      rme         Renamed from iekfupdateaug.m to update_aug_iekf.m
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Modified to work with preallocated TheJournal
%    11-26-2004      rme         Modified to loop over meas_t and lump into a single measurement.

global TheJournal;

% shorthand index
Nv   = TheJournal.Index.Nv;   % size of vehicle state
Naug = TheJournal.Index.Naug; % size of augmented state
Xa_i = TheJournal.Index.Xa_i; % "all" state index

% pointers into TheJournal
mu_0    = TheJournal.Ekf.mu(Xa_i);
Sigma_0 = TheJournal.Ekf.Sigma(Xa_i,Xa_i);

% setup IEKF loop
maxIterations = 1;
mu_i    = mu_0;
[Haug_i,Kaug_i,nu_i,S_i] = deal([]);
error_i = inf;
error_stats = zeros(maxIterations,1);
for ii=1:maxIterations;
  Haug_iprev = Haug_i; % store Jacobian from previous iteration
  
  % loop over meas_t and aggregate measurements
  %-----------------------------------------------
  [z_predict_i,z_fix,R_fix,Haug_i] = deal(cell(size(meas_t)));
  for jj=1:length(meas_t);
    % evaluate the predicted measurement and the Jacobian of the observation
    % model.  allow passing of optional arguments to observation function
    %--------------------------------------------
    % measurement prediction
    [z_predict_i{jj},z_fix{jj},R_fix{jj},Haug_i{jj}] = ...
	feval(meas_t(jj).omfhandle, mu_i, TheJournal.Index, ...
	      meas_t(jj).z, meas_t(jj).R, meas_t(jj).Xfi_i, meas_t(jj).Xfj_i, meas_t(jj).varargin{:});
    
    if isempty(Haug_i{jj});    
      % allocate a sparse Jacobian pattern with m*Nv*2 non-zero elements
      % m is the size of the measurement vector
      % Nv is the number of variables associated with a delayed state
      % 2 is the number of delayed states involved in a relative pose measurement
      m = length(meas_t(jj).z);
      Jpattern = spalloc(m,Naug,m*Nv*2);
      Jpattern(:,[meas_t.Xfi_i, meas_t.Xfj_i]) = 1;

      % numerically evaluate the sparse Jacobian of the observation model w.r.t. the state vector
      fprintf('==>%s: Computing numerical Jacobian for %s\n',mfilename,meas_t(jj).omfhandleString);
      Haug_i{jj} = numerical_jacobian(meas_t(jj).omfhandle, mu_i, z_predict_i{jj}, Jpattern, ...
				      TheJournal.Index, meas_t(jj).z, meas_t(jj).R, ...
				      meas_t(jj).Xfi_i, meas_t(jj).Xfj_i, meas_t(jj).varargin{:});
    end;
  end; % for jj=1:length(meas_t)
  
  % lump into a single large measurement vector
  %-----------------------------------------------
  z_predict_i = vertcat(z_predict_i{:});
  z_fix       = vertcat(z_fix{:});
  R_fix       = sparse(blkdiag(R_fix{:}));
  Haug_i      = sparse(vertcat(Haug_i{:}));
  
  % find indices of nonzero columns in Jacobian
  %-----------------------------------------------
  nzi = find(any(Haug_i,1));
  
  % iterative innovation and covariance
  %--------------------------------------------
  nu_iprev = nu_i;     % store innovation from previous iteration
  S_iprev  = S_i;      % store innovation covariance from previous iteration
  nu_i = z_fix - z_predict_i - Haug_i(:,nzi)*(mu_0(nzi) - mu_i(nzi));
  S_i  = R_fix + spdproduct(Sigma_0(nzi,nzi),Haug_i(:,nzi)');
  
  % iterative Kalman gain
  %--------------------------------------------
  Kaug_iprev = Kaug_i; % store gain matrix from previous iteration
  Kaug_i = Sigma_0*Haug_i'*spdinverse(S_i);
  Kaug_i = sparse(Kaug_i);

  % iterative state update
  %--------------------------------------------
  mu_iprev = mu_i; % store updated state from previous iteration
  mu_i = mu_0 + Kaug_i*nu_i;
  
  % check iterative terminate criteria
  %--------------------------------------------
  TOL = 1e-12;  % (10^-6)^2
  error_iprev = error_i;
  evec = (mu_i - mu_iprev);
  error_i = evec'*evec;
  error_stats(ii) = error_i;
  if error_i < TOL;
    break; % exit loop
  elseif error_i > error_iprev;
    % increase in error, terminate with previouse iteration estimate    
    mu_i   = mu_iprev;
    Haug_i = Haug_iprev;
    Kaug_i = Kaug_iprev;
    nu_i   = nu_iprev;
    S_i    = S_iprev;
    break; % exit loop
  end;

end; % for ii=1:maxIterations

% store mean update
%-------------------------------------------
TheJournal.Ekf.mu(Xa_i) = mu_i;

% print iterative error stats
%-------------------------------------------
for kk=1:ii;
  fprintf('==>%s: IEKF i=%d  e=%.3e  ', mfilename, kk, error_stats(kk));
  for jj=1:length(meas_t);
    fprintf('%s ',meas_t(jj).omfhandleString);
  end;
  fprintf('\n');
end;

% covariance update (based on Joseph form)
%--------------------------------------------
persistent Iaug;
if Naug ~= size(Iaug,1);
  % allocate a sparse identity matrix to match size of augment state vector
  Iaug = speye(Naug);
end;
IminusKH = (Iaug-Kaug_i*Haug_i);
TheJournal.Ekf.Sigma(Xa_i,Xa_i) =  ...
    spdproduct(Sigma_0,IminusKH') + spdproduct(R_fix,Kaug_i');
