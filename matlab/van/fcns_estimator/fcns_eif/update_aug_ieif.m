function [] = update_aug_ieif(meas_t,TheConfig)
%function [] = update_aug_ieif(meas_t,TheConfig)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created from update_nav_eif.m
%    10-26-2004      rme         Added Cholesky decompostion to make
%                                update more numerically stable w.r.t.
%                                information matrix symmetry
%    11-02-2004      rme         Modified as part of major code reorganization.
%    11-03-2004      rme         Modified to be more efficient by only updating
%                                state elements corresponding to nonzero Jacobian
%    11-26-2004      rme         Modified to loop over meas_t and lump into a single measurement.
%    11-27-2004      rme         Added LcholTainted flag

global TheJournal;

% shorthand index
Nv   = TheJournal.Index.Nv;   % size of vehicle state
Naug = TheJournal.Index.Naug; % size of state vector
Xa_i = TheJournal.Index.Xa_i; % "all" state index

% pointers into TheJournal
mu     = TheJournal.Eif.mu(Xa_i);
eta    = TheJournal.Eif.eta(Xa_i);
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
[mu_0,eta_0,Lambda_0] = deal(mu,eta,Lambda);

maxIterations = 1000;
try
for kk=1:maxIterations;
  kk
  for jj=1:length(meas_t);
    % evaluate the predicted measurement and the Jacobian of the observation
    % model.  allow passing of optional arguments to observation function.
    % note that the observation function takes the measurement "z" as an
    % argument and also returns it as an output.  in most cases the output z
    % argument will remain unchanged from the input.  however this
    % input/output capability facilitates angular unwrapping of raw attitude
    % measurements to be consistent with the predicted unwrapped state estimate.
    %----------------------------------------------------------------------------
    % measurement prediction returns analytical Jacobian
    clear z_predict z_fix R_fix Haug;
    [z_predict{jj},z_fix{jj},R_fix{jj},Haug{jj}] = ...
	feval(meas_t(jj).omfhandle, mu, TheJournal.Index, meas_t(jj).z, meas_t(jj).R, ...
	      meas_t(jj).Xfi_i, meas_t(jj).Xfj_i, meas_t(jj).varargin{:});
  
    if isempty(Haug{jj});
      % allocate a sparse Jacobian pattern with m*Nv*2 non-zero elements
      % m  is the size of measurement vector
      % Nv is the number of variables associated with a delayed state
      % 2  is the number of delayed states involved in a relative pose measurement
      m = length(meas_t(jj).z);
      Jpattern = spalloc(m,Naug,m*Nv*2);
      Jpattern(:,[meas_t(jj).Xfi_i, meas_t(jj).Xfj_i]) = 1;
  
      % numerically evaluate the sparse Jacobian of the observation model w.r.t. the state vector
      fprintf('==>%s: Computing numerical Jacobian for %s\n',mfilename,meas_t(jj).omfhandleString);
      Haug{jj} = numerical_jacobian(meas_t(jj).omfhandle, mu, z_predict{jj}, Jpattern, ...
				    TheJournal.Index, meas_t(jj).z, meas_t(jj).R, ...
				    meas_t(jj).Xfi_i, meas_t(jj).Xfj_i, meas_t(jj).varargin{:});
    end;
  end; % for jj=1:length(meas_t)

  % lump into a single large measurement vector
  %-----------------------------------------------
  z_predict = vertcat(z_predict{:});
  z_fix     = vertcat(z_fix{:});
  R_fix     = sparse(blkdiag(R_fix{:}));
  Haug      = sparse(vertcat(Haug{:}));

  % find indices of nonzero columns in Jacobian
  %-----------------------------------------------
  nzi = find(any(Haug,1));

  % innovation
  %--------------------------------------------
  nu = z_fix - (z_predict - Haug(:,nzi)*mu(nzi));
  %nu = z_fix - (z_predict - Haug(:,nzi)*(mu_0(nzi) - mu(nzi)));
  
  % information vector update
  %--------------------------------------------
  R_inv = spdinverse(R_fix);
  TheJournal.Eif.eta(nzi) = eta_0(nzi) + Haug(:,nzi)'*R_inv*nu;

  % information matrix update
  %---------------------------------------------
  TheJournal.Eif.Lambda(nzi,nzi) = Lambda_0(nzi,nzi) + spdproduct(R_inv,Haug(:,nzi));


  % recover updated mean
  %---------------------------------------------
  if TheConfig.Estimator.useFullStateRecovery;
    recover_state_full(1); %TheJournal
  else;
    recover_state_pcg(TheConfig.Estimator.pcgMaxIter,TheConfig.Estimator.groupSize,1); %TheJournal
  end;
  mu = TheJournal.Eif.mu(Xa_i);
  
end;
catch
  keyboard;
end;

% set preconditioner outdated flag
TheJournal.Eif.LcholTainted = true;
