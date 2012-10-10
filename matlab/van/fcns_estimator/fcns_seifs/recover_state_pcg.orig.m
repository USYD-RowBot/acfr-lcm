function [] = recover_state_pcg(maxIter,groupSize)
%function [] = recover_state_pcg(maxIter,groupSize)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-18-2004      rme         Created and written.
%    10-21-2004      rme         Added check for convergence
%    10-26-2004      rme         Added Gaussian Elimination solution for "small" systems
%    10-28-2004      rme         Major code reorganization
%    11-02-2004      rme         Modified to use converged flag.
%                                Modified to use preallocated TheJournal.
%    11-04-2004      rme         Recompute cholinc if size doesn't match Lambda due
%                                to state augmentation
%    11-05-2004      rme         Fixed bug in setting converged flag to be true
%    11-08-2004      rme         Added setTerminal color.
%    11-11-2004      rme         Added full state recovery switch

global TheJournal;

% shorthand index
Nf   = TheJournal.Index.Nf;
Nv   = TheJournal.Index.Nv;
Naug = TheJournal.Index.Naug;
Xa_i = TheJournal.Index.Xa_i; % "all" state index

% pointers into TheJournal;
mu     = TheJournal.Eif.mu(Xa_i);
eta    = TheJournal.Eif.eta(Xa_i);
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
Lchol  = TheJournal.Eif.Lchol;

% if state vector is small, just solve via Gaussian Elimination
if ( Nf+1 < groupSize );
  fprintf('==>%s: full state recovery... ',mfilename);
  aclock('tic');
  recover_state_full; %TheJournal
  fprintf('done, dt = %.3f\n',aclock('toc'));
  return;
end;

% update our preconditioner and assign pointer
updatePreConditioner; %TheJournal
Lchol = TheJournal.Eif.Lchol;

% do PCG
fprintf('==>%s: Attempt PCG state recovery... ',mfilename);
aclock('tic');
tol = 1e-8;
[TheJournal.Eif.mu(Xa_i),flag] = pcg(Lambda, eta, tol, maxIter, Lchol', Lchol, mu);
fprintf('done, dt = %.3f\n',aclock('toc'));

% warn about convergence
%if flag > 0;
%  setTerminal('warning');
%  fprintf('***%s: PCG did not converge within MAXIT=%d\n',mfilename,MAXIT);
%  setTerminal('restore');
%end
