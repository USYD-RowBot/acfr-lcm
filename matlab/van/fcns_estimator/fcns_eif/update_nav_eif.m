function [] = update_nav_eif(meas_t,TheConfig)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-08-2004      rme         Created from iekfupdatenav.m
%    10-26-2004      rme         Added Cholesky decompostion to make
%                                update more numerically stable w.r.t.
%                                information matrix symmetry
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Added spdinverse.m & spdproduct.m
%    11-26-2004      rme         Modified to loop over meas_t and lump into a single measurement.
%    11-27-2004      rme         Added LcholTainted flag

global TheJournal;
  
% shorthand index into augmented state vector
Xv_i = TheJournal.Index.Xv_i;  % vehicle state index

% pointers into TheJournal
mu_x      = TheJournal.Eif.mu(Xv_i);
eta_x     = TheJournal.Eif.eta(Xv_i);
Lambda_xx = TheJournal.Eif.Lambda(Xv_i,Xv_i);

for jj=1:length(meas_t);
  % evaluate the predicted measurement and the Jacobian of the observation
  % model.  allow passing of optional arguments to observation function.
  % note that the observation function takes the measurement "z" as an
  % argument and also returns it as an output.  in most cases the output z
  % argument will remain unchanged from the input.  however this
  % input/output capability facilitates angular unwrapping of raw attitude
  % measurements to be consistent with the predicted unwrapped state estimate.
  %----------------------------------------------------------------------------
  % measurement prediction returns Jacobian
  [z_predict{jj},z_fix{jj},R_fix{jj},Hv{jj}] = ...
      feval(meas_t(jj).omfhandle, mu_x, TheJournal.Index, ...
	    meas_t(jj).z, meas_t(jj).R, meas_t(jj).varargin{:});
  
  if isempty(Hv{jj});
    % numerically evaluate the Jacobian of the observation model w.r.t. vehicle state vector
    disp('Computing numerical jacobian Hv');
    Hv{jj} = numerical_jacobian(meas_t(jj).omfhandle, mu_x, z_predict{jj}, ...
				[], TheJournal.Index, meas_t(jj).z, meas_t(jj).R, ...
				meas_t(jj).varargin{:});
  end;
end; % for jj=1:length(meas_t)

% lump into a single large measurement vector
%-----------------------------------------------
z_predict = vertcat(z_predict{:});
z_fix     = vertcat(z_fix{:});
R_fix     = sparse(blkdiag(R_fix{:}));
Hv        = sparse(vertcat(Hv{:}));

% innovation
%--------------------------------------------
nu = z_fix - (z_predict - Hv*mu_x);

% information vector update
%--------------------------------------------
R_inv = spdinverse(R_fix);
TheJournal.Eif.eta(Xv_i) = eta_x + Hv'*R_inv*nu;

% information matrix update
%--------------------------------------------- 
TheJournal.Eif.Lambda(Xv_i,Xv_i) = Lambda_xx + spdproduct(R_inv,Hv);

% set preconditioner outdated flag
TheJournal.Eif.LcholTainted = true;
