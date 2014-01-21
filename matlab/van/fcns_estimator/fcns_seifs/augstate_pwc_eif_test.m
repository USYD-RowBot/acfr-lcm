function [] = augstate_pwc_eif_test(dt,TheConfig)
%function [] = augstate_pwc_eif(dt,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created and written.
%    10-14-2004      rme         Added additional comments.
%                                Changed to *append* new state to state vector
%    10-28-2004      rme         Major code reorganization.

global TheJournal;

% shorthand index into augmented state vector
Nv    = TheJournal.Index.Nv;
Xvp_i = TheJournal.Index.Xv_i; % previous vehicle state index x(t)
Xvc_i = Xvp_i + Nv;            % current  vehicle state index x(t+1)
Xf_i  = TheJournal.Index.Xf_i; % feature state index

% 1st order piecewise-constant discrete time model
[Fk,Bk,uk,Qk] = pwcContinuousToDiscrete(TheConfig.ProcessModel.pmfhandle, TheJournal.Eif.mu(Xvp_i), ...
					TheJournal.Index, [], TheConfig.ProcessModel.Qv, dt);

% pointers into TheJournal
mu_x     = TheJournal.Eif.mu(Xvp_i);          % robot(t)
mu_y     = Fk*mu_x + Bk*uk;                   % robot(t+1)
Sigma    = spdinverse(TheJournal.Eif.Lambda);
Sigma_xx = Sigma(Xvp_i,Xvp_i);
Sigma_xM = Sigma(Xvp_i,Xf_i);  % robot(t) / Map

%~~~~~~~~~~~~~~~ state augmentation ~~~~~~~~~~~~~~~~~~~~~~~~~~
% append the state vector with the time projected state x(t+1)
%-------------------------------------------------------------
% mean vector
TheJournal.Eif.mu(Xvc_i) = mu_y;

% covariance matrix
Sigma(Xvc_i,Xvc_i) = spdproduct(Sigma_xx,Fk')+Qk;     % Sigma_yy
Sigma(Xvc_i,Xvp_i) = Fk*Sigma_xx;                        % Sigma_yx
Sigma(Xvc_i,Xf_i)  = Fk*Sigma_xM;                        % Sigma_yM
Sigma(Xvp_i,Xvc_i) =    Sigma(Xvc_i,Xvp_i)'; % Sigma_xy
Sigma(Xf_i,Xvc_i)  =    Sigma(Xvc_i,Xf_i)';  % Sigma_My

TheJournal.Eif.Lambda = spdinverse(Sigma);
TheJournal.Eif.eta    = TheJournal.Eif.Lambda * TheJournal.Eif.mu;

