function [] = recover_state_full();
%function [] = recover_state_full();
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-12-2004      rme         Created and written.
%    11-27-2004      rme         Modified computation to use spdmldivide.m  

global TheJournal;

% pointers into TheJournal
Xa_i = TheJournal.Index.Xa_i;
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
eta    = TheJournal.Eif.eta(Xa_i);

% full state recovery via Gaussian elimination
TheJournal.Eif.mu(Xa_i) = Lambda\eta;
