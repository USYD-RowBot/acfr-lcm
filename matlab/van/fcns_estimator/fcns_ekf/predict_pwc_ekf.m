function [] = predict_pwc_ekf(dt,TheConfig)
%function [] = predict_pwc_ekf(dt,TheConfig)  
%
% Approximate the nonlinear process model as piecewise-constant
% using a 1st order Taylor series representation
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-29-2004      rme         Created from ekfpredict.m
%    07-15-2004      rme         Modified to accept args from ct2dt.m
%    09-09-2004      rme         Renamed file from ekfpredic_pwc.m to predict_pwc_ekf.m
%                                Renamed variables Xaug to mu, Paug to Sigma
%    10-28-2004      rme         Major code reorganization.
%    10-29-2004      rme         Fixed a typo in covariance update.
%    11-01-2004      rme         Modified to use spdproduct.m
  
global TheJournal;

% shorthand index into augmented state vector
Xv_i = TheJournal.Index.Xv_i; % vehicle state index
Xf_i = TheJournal.Index.Xf_i; % feature state index

% 1st order piecewise-constant DT model
[Fk,Bk,uk,Qk] =  pwcContinuousToDiscrete(TheConfig.ProcessModel.pmfhandle, TheJournal.Ekf.mu(Xv_i), ...
					 TheJournal.Index, [], TheConfig.ProcessModel.Qv, dt);      

% pointers into TheJournal
mu_x     = TheJournal.Ekf.mu(Xv_i);         % robot(t)
Sigma_xx = TheJournal.Ekf.Sigma(Xv_i,Xv_i);
Sigma_xM = TheJournal.Ekf.Sigma(Xv_i,Xf_i); % robot(t), Map

%~~~~~~~~~~~~~~~ propagate statistics ~~~~~~~~~~~~~~~~~~~~~
% time projected state x(t) -> x(t+1)
%----------------------------------------------------------
% time propogated mean
TheJournal.Ekf.mu(Xv_i) = Fk*mu_x + Bk*uk;              % mu_x(t+1)

% time propogated covariance
TheJournal.Ekf.Sigma(Xv_i,Xv_i) = Qk + spdproduct(Sigma_xx,Fk'); % Sigma_xx
TheJournal.Ekf.Sigma(Xv_i,Xf_i) = Fk*Sigma_xM;                               % Sigma_xM
TheJournal.Ekf.Sigma(Xf_i,Xv_i) = TheJournal.Ekf.Sigma(Xv_i,Xf_i)';          % Sigma_Mx
