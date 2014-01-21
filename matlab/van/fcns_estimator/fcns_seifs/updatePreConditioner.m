function updatePreConditioner();
%function updatePreConditioner();  
%
%  Computes a preconditioner suitable for PCG.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-27-2004      rme         Created and written.  
  
global TheJournal;

% pointers into TheJournal
Xa_i = TheJournal.Index.Xa_i;
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);

if TheJournal.Eif.LcholTainted;
  TheJournal.Eif.LcholTainted = false;  
  fprintf('==>%s: Incomplete Cholesky factorization... ',mfilename);
  aclock('tic');
  TheJournal.Eif.Lchol = cholinc(Lambda,1e-3);
  fprintf('done, dt = %.3f\n',aclock('toc'));
end;
