function [] = augstate_pwc_ekf(dt,TheConfig)
%function [] = augstate_pwc_ekf(dt,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created and written.
%    10-14-2004      rme         Added additional comments.
%                                Changed to *append* new state to state vector
%    10-28-2004      rme         Major code reorganization.
%    11-01-2004      rme         Modified to use spdproduct.m

global TheJournal;

% shorthand index into augmented state vector
Nv    = TheJournal.Index.Nv;
Xvp_i = TheJournal.Index.Xv_i; % previous vehicle state index x(t)
Xvc_i = Xvp_i + Nv;            % current vehicle state index  x(t+1)
Xf_i  = TheJournal.Index.Xf_i; % feature state index

% 1st order piecewise-constant DT model
[Fk,Bk,uk,Qk] =  pwcContinuousToDiscrete(TheConfig.ProcessModel.pmfhandle, TheJournal.Ekf.mu(Xvp_i), ...
					 TheJournal.Index, [], TheConfig.ProcessModel.Qv, dt);

% pointers into TheJournal
mu_x     = TheJournal.Ekf.mu(Xvp_i);          % robot(t)
mu_y     = Fk*mu_x + Bk*uk;                   % robot(t+1)
Sigma_xx = TheJournal.Ekf.Sigma(Xvp_i,Xvp_i);
Sigma_xM = TheJournal.Ekf.Sigma(Xvp_i,Xf_i);  % robot(t), Map

%~~~~~~~~~~~~~~~ state augmentation ~~~~~~~~~~~~~~~~~~~~~~~~~~
% append the state vector with the time projected state x(t+1)
%-------------------------------------------------------------
% mean vector
TheJournal.Ekf.mu(Xvc_i) = mu_y;

% covariance matrix
TheJournal.Ekf.Sigma(Xvc_i,Xvc_i) = Qk + spdproduct(Sigma_xx,Fk'); % Sigma_yy
TheJournal.Ekf.Sigma(Xvc_i,Xvp_i) = Fk*Sigma_xx;                               % Sigma_yx
TheJournal.Ekf.Sigma(Xvp_i,Xvc_i) = TheJournal.Ekf.Sigma(Xvc_i,Xvp_i)';        % Sigma_xy
TheJournal.Ekf.Sigma(Xvc_i,Xf_i)  = Fk*Sigma_xM;                               % Sigma_yM
TheJournal.Ekf.Sigma(Xf_i,Xvc_i)  = TheJournal.Ekf.Sigma(Xvc_i,Xf_i)';         % Sigma_My
