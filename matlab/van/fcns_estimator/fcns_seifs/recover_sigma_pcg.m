function [] = recover_sigma_pcg(maxIter,groupSize);
%function [] = recover_sigma_pcg(maxIter,groupSize);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-27-2004      rme         Created and written from recover_state_pcg.m

global TheJournal;

% shorthand index
Np   = TheJournal.Index.Np;
Nf   = TheJournal.Index.Nf;
Naug = TheJournal.Index.Naug;
fn   = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;            % pose element index
Xa_i = TheJournal.Index.Xa_i;            % "all" state index
Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i); % most recent delayed state pose index

% pointers into TheJournal;
mu       = TheJournal.Eif.mu(Xa_i);
eta      = TheJournal.Eif.eta(Xa_i);
Lambda   = TheJournal.Eif.Lambda(Xa_i,Xa_i);
SigmaCol = TheJournal.Eif.SigmaCol(Xa_i,:);

% basis vectors of most recent delayed state pose elements
E = spalloc(Naug,Np,Np);
E(Xf_i,:) = speye(Np);

% if state vector is small, just solve via Gaussian Elimination
if ( Nf+1 < groupSize );
  fprintf('==>%s: full state recovery... ',mfilename);
  aclock('tic');
  recover_sigma_full; %TheJournal
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
for k=1:Np;
  [TheJournal.Eif.SigmaCol(Xa_i,k),flag] = pcg(Lambda, E(:,k), tol, maxIter, Lchol', Lchol, SigmaCol(:,k));
end;
% ensure autocovariance is symmetric
TheJournal.Eif.SigmaCol(Xf_i,:) = symmetrize(TheJournal.Eif.SigmaCol(Xf_i,:));
fprintf('done, dt = %.3f\n',aclock('toc'));

% warn about convergence
%if flag > 0;
%  setTerminal('warning');
%  fprintf('***%s: PCG did not converge within MAXIT=%d\n',mfilename,MAXIT);
%  setTerminal('restore');
%end
