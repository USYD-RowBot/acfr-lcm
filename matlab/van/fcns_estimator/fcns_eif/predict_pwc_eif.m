function [] = predict_pwc_eif(dt,TheConfig)
%function [] = predict_pwc_eif(dt,TheConfig)  
%
% Approximate the nonlinear process model as piecewise-constant
% using a 1st order Taylor series representation
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-08-2004      rme         Created and written.
%    10-26-2004      rme         Added Cholesky decomposition to make prediction
%                                more numerically stable w.r.t. preserving matrix symmetry
%    10-28-2004      rme         Major code reorganization.
%    11-01-2004      rme         Modified to use spdproduct.m & spdinverse.m
%    11-02-2004      rme         Speed up execution by only indexing "active" features
%    11-27-2004      rme         Added LcholTainted flag
%    01-08-2005      rme         Added persistent Xv_i_old to reduce time spent calculating
%                                active index elements ai.

global TheJournal;
  
% shorthand index into augmented state vector
Xv_i  = TheJournal.Index.Xv_i; % vehicle state index
Xf_i  = TheJournal.Index.Xf_i; % feature state index
persistent Xv_i_old ai;
if isempty(Xv_i_old) || ~all(Xv_i==Xv_i_old);
  ai    = find(any(TheJournal.Eif.Lambda(Xv_i,Xf_i),1)); % index of "active" elements
  Xv_i_old = Xv_i;
end;
Xaf_i = Xf_i(ai);              % active feature state index

% 1st order piecewise-constant discrete time model
[Fk,Bk,uk,Qk] = pwcContinuousToDiscrete(TheConfig.ProcessModel.pmfhandle, TheJournal.Eif.mu(Xv_i), ...
					TheJournal.Index, [], TheConfig.ProcessModel.Qv, dt);

% pointers to some useful terms
mu_x      = TheJournal.Eif.mu(Xv_i);
eta_x     = TheJournal.Eif.eta(Xv_i);
eta_M     = TheJournal.Eif.eta(Xaf_i,1);             % "active" portion of map
Lambda_xx = TheJournal.Eif.Lambda(Xv_i,Xv_i);
Lambda_xM = TheJournal.Eif.Lambda(Xv_i,Xaf_i);       % shared information with "active" portion of map
Lambda_MM = TheJournal.Eif.Lambda(Xaf_i,Xaf_i);      % "active" portion of map

% precompute some useful terms
Qk_inv        = spdinverse(Qk);
Lambda_xx_inv = spdinverse(Lambda_xx);
Phi_inv       = Qk + spdproduct(Lambda_xx_inv,Fk');
Phi           = spdinverse(Phi_inv);
Omega         = Lambda_xx + spdproduct(Qk_inv,Fk);
Omega_inv     = spdinverse(full(Omega));

%~~~~~~~~~~~~~~~ propagate statistics ~~~~~~~~~~~~~~~~~~~~~
% time projected state x(t) -> x(t+1)
%----------------------------------------------------------
% time propagated mean vector
TheJournal.Eif.mu(Xv_i)  = Fk*mu_x + Bk*uk;

% time propagated information vector
TheJournal.Eif.eta(Xv_i)  = Qk_inv*Fk*Omega_inv*eta_x + Phi*Bk*uk;                   % eta_x(t+1)
TheJournal.Eif.eta(Xaf_i) = eta_M - Lambda_xM'*Omega_inv*(eta_x - Fk'*Qk_inv*Bk*uk); % eta_M

% time propagated information matrix
TheJournal.Eif.Lambda(Xv_i,Xv_i)   = Phi;                                                     % Lambda_xx
TheJournal.Eif.Lambda(Xv_i,Xaf_i)  = Qk_inv*Fk*Omega_inv*Lambda_xM;                           % Lambda_xM
TheJournal.Eif.Lambda(Xaf_i,Xv_i)  = TheJournal.Eif.Lambda(Xv_i,Xaf_i)';                      % Lambda_Mx
TheJournal.Eif.Lambda(Xaf_i,Xaf_i) = Lambda_MM - spdproduct(Omega_inv,Lambda_xM); % Lambda_MM

% set preconditioner outdated flag
TheJournal.Eif.LcholTainted = true;
