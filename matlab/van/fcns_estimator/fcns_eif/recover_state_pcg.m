function [] = recover_state_pcg(maxIter,groupSize,evalflag)
%function [] = recover_state_pcg(maxIter,groupSize,evalflag)
%
% evalflag = 1: solve mu
%            2: solve for SigmaCol
%            3: solve for mu & SigmaCol
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
%    11-27-2004      rme         Added option for SigmaCol recovery.

global TheJournal;

% shorthand index
Nf   = TheJournal.Index.Nf;
Xa_i = TheJournal.Index.Xa_i;            % "all" state index

% pointers into TheJournal;
eta    = TheJournal.Eif.eta(Xa_i);
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);

% if state vector is small, just solve via Gaussian Elimination
if ( Nf+1 < groupSize );
  fprintf('==>%s: type %d full state recovery... ',mfilename,evalflag);
  aclock('tic');
  recover_state_full(evalflag); %TheJournal
  fprintf('done, dt = %.3f\n',aclock('toc'));
  return;
end;

% update our preconditioner
if TheJournal.Eif.LcholTainted;
  TheJournal.Eif.LcholTainted = false;  
  fprintf('==>%s: Incomplete Cholesky factorization... ',mfilename);
  aclock('tic');
  TheJournal.Eif.Lchol = cholinc(Lambda,1e-3);
  fprintf('done, dt = %.3f\n',aclock('toc'));
end;

% do PCG
fprintf('==>%s: type %d PCG state recovery... ',mfilename,evalflag);
aclock('tic');
tol = 1e-12/norm(eta);
switch evalflag
case 1;
 private_pcg_mu(tol,maxIter);
case 2;
 private_pcg_sigma(tol,maxIter);
case 3;
 private_pcg_mu(tol,maxIter);
 private_pcg_sigma(tol,maxIter);
otherwise;
 error('unkown evalflag');
end; % switch evalflag
fprintf('done, dt = %.3f\n',aclock('toc'));


%============================================================================
function private_pcg_mu(tol,maxIter)
global TheJournal;
% shorthand index
Xa_i = TheJournal.Index.Xa_i;            % "all" state index

% pointers into TheJournal;
mu     = TheJournal.Eif.mu(Xa_i);
eta    = TheJournal.Eif.eta(Xa_i);
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
Lchol  = TheJournal.Eif.Lchol;

[TheJournal.Eif.mu(Xa_i),flag] = pcg(Lambda, eta, tol, maxIter, Lchol', Lchol, mu);

%============================================================================
function private_pcg_sigma(tol,maxIter)
global TheJournal;

% shorthand index
Np   = TheJournal.Index.Np;
Naug = TheJournal.Index.Naug;
fn   = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;            % pose element index
Xa_i = TheJournal.Index.Xa_i;            % "all" state index
Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i); % most recent delayed state pose index

% pointers into TheJournal;
Lambda   = TheJournal.Eif.Lambda(Xa_i,Xa_i);
SigmaCol = TheJournal.Eif.SigmaCol(Xa_i,:);
Lchol    = TheJournal.Eif.Lchol;

% basis vectors of most recent delayed state pose elements
E = spalloc(Naug,Np,Np);
E(Xf_i,:) = speye(Np);

for ii=1:Np;
  [TheJournal.Eif.SigmaCol(Xa_i,ii),flag] = pcg(Lambda, E(:,ii), tol, maxIter, Lchol', Lchol, SigmaCol(:,ii));
end;
% ensure autocovariance is symmetric
TheJournal.Eif.SigmaCol(Xf_i,:) = symmetrize(TheJournal.Eif.SigmaCol(Xf_i,:));
