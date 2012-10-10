function Sigma = cell2Sigma(TheJournal,SigmaCell);
%function Sigma = sigmaCI(TheJournal,SigmaCell);

% assembles the block diag covariance matrix associated with delayed
% state poses.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-19-2005      rme         Created and written.m
%    01-20-2005      rme         Modified spalloc call.

Naug  = TheJournal.Index.Naug;
Np    = TheJournal.Index.Np;
Xf_ii = TheJournal.Index.Xf_ii;
Xp_i  = TheJournal.Index.Xp_i;
Xa_i  = TheJournal.Index.Xa_i;
Nf    = length(Xf_ii)-1; % excluding robot

Sigma = spalloc(Naug,Naug,Nf*Np^2+Naug*Np*2);
for fn=1:Nf;
  Xfi_i = Xf_ii{fn}(Xp_i);
  
  % extract block covariance
  if fn == TheJournal.Index.fn;
    Sigma(Xfi_i,:) = TheJournal.Eif.SigmaCol(Xa_i,:)';
    Sigma(:,Xfi_i) = TheJournal.Eif.SigmaCol(Xa_i,:);
  else;
    Sigma(Xfi_i,Xfi_i) = SigmaCell{fn};
  end;
end;
