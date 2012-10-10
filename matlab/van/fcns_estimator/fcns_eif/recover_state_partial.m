function [] = recover_state_partial(Xl_i)
%function [] = recover_state_partial(Xl_i)  
%
%  X_i is the index of the state elements we wish to "recover"
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-2004         rme         Created and written.
%    10-17-2004      rme         Return only mu_hat_ii subelements
%    10-28-2004      rme         Major code reorganization.
%    11-11-2004      rme         removed full state recovery code
%    11-12-2004      rme         Redefine input arguments
%    11-17-2004      rme         Modified computation to use the whole block
%                                row Lambda(Xl_i,:) eliminating indexing issues
%    11-27-2004      rme         Modified computation to use spdmldivide.m
%    01-04-2005      rme         Removed spdmldivide.m && added persistence of non-zero
%                                element index.
%    01-08-2005      rme         Removed pointers into TheJournal to speed up computation.
  
global TheJournal;

% using the terminology of the ICRA05 paper, Xl_i defines the index
% of the "local" map we wish to recover, and Xb_i defines the "benign"
% portion of the map we hold fixed.

% generate the "benign" state index
%Xb_i       = Xa_i;
%Xb_i(Xl_i) = [];

% nonzero element index
persistent Xl_i_prev nz;
if isempty(Xl_i_prev) || prod(size(Xl_i))~=prod(size(Xl_i_prev)) || ~all(Xl_i_prev==Xl_i);
  nz = find(any(TheJournal.Eif.Lambda(Xl_i,:)));
  Xl_i_prev = Xl_i;
end;

% partial state recovery
TheJournal.Eif.mu(Xl_i) = TheJournal.Eif.mu(Xl_i) + ...
    TheJournal.Eif.Lambda(Xl_i,Xl_i) \ ( TheJournal.Eif.eta(Xl_i)-TheJournal.Eif.Lambda(Xl_i,nz)*TheJournal.Eif.mu(nz) );
