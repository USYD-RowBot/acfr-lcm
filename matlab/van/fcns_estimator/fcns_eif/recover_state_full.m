function [] = recover_state_full(evalflag);
%function [] = recover_state_full(evalflag);
%
% evalflag = 1: solve mu
%            2: solve for SigmaCol
%            3: solve for mu & SigmaCol  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-12-2004      rme         Created and written.
%    11-27-2004      rme         Added optional SigmaCol recovery.
%    01-18-2005      rme         Cleaned-up a bit.

global TheJournal;

% shorthand index
Np   = TheJournal.Index.Np;
Naug = TheJournal.Index.Naug;
fn   = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;            % pose element index
Xa_i = TheJournal.Index.Xa_i;            % "all" state index
Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i); % most recent delayed state pose index

% pointers into TheJournal
Xa_i     = TheJournal.Index.Xa_i;
Lambda   = TheJournal.Eif.Lambda(Xa_i,Xa_i);
eta      = TheJournal.Eif.eta(Xa_i);

if evalflag > 1;
  % basis vectors of most recent delayed state pose elements
  E = spalloc(Naug,Np,Np);
  E(Xf_i,:) = speye(Np);
end;

% full state recovery via Gaussian elimination
switch evalflag;
case 1; % mu
 TheJournal.Eif.mu(Xa_i) = Lambda \ eta;
case 2; % SigmaCol
 TheJournal.Eif.SigmaCol(Xa_i,:) = Lambda \ E;
case 3; % mu, SigmaCol
 B = [eta,E];
 tmp = Lambda \ B;
 TheJournal.Eif.mu(Xa_i) = tmp(:,1);
 TheJournal.Eif.SigmaCol(Xa_i,:) = tmp(:,2:end);
otherwise;
 error('uknown evalflag');
end; % switch evalflag

if evalflag > 1;
  % ensure autocovariance is symmetric
  TheJournal.Eif.SigmaCol(Xf_i,:) = symmetrize(TheJournal.Eif.SigmaCol(Xf_i,:));
end;
