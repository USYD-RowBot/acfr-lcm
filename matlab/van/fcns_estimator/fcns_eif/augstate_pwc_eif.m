function [] = augstate_pwc_eif(dt,TheConfig)
%function [] = augstate_pwc_eif(dt,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created and written.
%    10-14-2004      rme         Added additional comments.
%                                Changed to *append* new state to state vector
%    10-28-2004      rme         Major code reorganization.
%    11-01-2004      rme         Modified to use symmericPosDefInverse.m & spdproduct.m
%    11-27-2004      rme         Added LcholTainted flag
%    11-27-2004      rme         Added replication of SigmaCol elements
  
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
mu_x      = TheJournal.Eif.mu(Xvp_i);           % robot(t)
mu_y      = Fk*mu_x + Bk*uk;                    % robot(t+1)
eta_x     = TheJournal.Eif.eta(Xvp_i);          
Lambda_xx = TheJournal.Eif.Lambda(Xvp_i,Xvp_i);

% precompute some useful terms
Qk_inv   = spdinverse(Qk);
Omega    = Lambda_xx + spdproduct(Qk_inv,Fk);

%~~~~~~~~~~~~~~~ state augmentation ~~~~~~~~~~~~~~~~~~~~~~~
% *append* state vector with the time projected state x(t+1)
%----------------------------------------------------------
% mean vector
TheJournal.Eif.mu(Xvc_i) = mu_y;

% information vector
TheJournal.Eif.eta(Xvc_i) = Qk_inv*Bk*uk;                          % eta_y
TheJournal.Eif.eta(Xvp_i) = eta_x - Fk'*TheJournal.Eif.eta(Xvc_i); % eta_x

% information matrix
TheJournal.Eif.Lambda(Xvc_i,Xvc_i) =  Qk_inv;                              % Lambda_yy
TheJournal.Eif.Lambda(Xvc_i,Xvp_i) = -Qk_inv*Fk;                           % Lambda_yx
TheJournal.Eif.Lambda(Xvp_i,Xvc_i) =  TheJournal.Eif.Lambda(Xvc_i,Xvp_i)'; % Lambda_xy
TheJournal.Eif.Lambda(Xvp_i,Xvp_i) =  Omega;                               % Lambda_xx

% set preconditioner outdated flag
TheJournal.Eif.LcholTainted = true;

% extend SigmaCol
TheJournal.Eif.SigmaCol(Xvc_i,:) = TheJournal.Eif.SigmaCol(Xvp_i,:);
