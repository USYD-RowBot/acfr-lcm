function [] = recover_sigma_full();
%function [] = recover_sigma_full();
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-27-2004      rme         Created and written.

global TheJournal;

% shorthand index
Np   = TheJournal.Index.Np;
Naug = TheJournal.Index.Naug;
fn   = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;            % pose element index
Xa_i = TheJournal.Index.Xa_i;            % "all" state index
Xf_i = TheJournal.Index.Xf_ii{fn}(Xp_i); % most recent delayed state pose index

% pointers into TheJournal;
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);

% basis vectors of most recent delayed state pose elements
E = spalloc(Naug,Np,Np);
E(Xf_i,:) = speye(Np);

% full state recovery via Gaussian elimination
TheJournal.Eif.SigmaCol(Xa_i,:) = Lambda\E;

% ensure autocovariance is symmetric
TheJournal.Eif.SigmaCol(Xf_i,:) = symmetrize(TheJournal.Eif.SigmaCol(Xf_i,:));
