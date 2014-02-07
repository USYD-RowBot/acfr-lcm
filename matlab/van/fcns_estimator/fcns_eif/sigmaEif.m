function Sigma = sigmaEif(TheJournal,flag);
%function Sigma = sigmaEif(TheJournal,flag);
% flag = {'all','block'}
% solve column-wise to recover the covariance matrix Sigma
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-06-2004      rme         Created and written.m
%    12-26-2004      rme         Added symmetrize.
%    01-15-2005      rme         Added option to recover entire covariance matrix.
%    01-12-2006      rme         Changed symmmd() to symamd().

if ~exist('flag','var') || isempty(flag); flag = 'block'; end;

switch lower(flag);
case 'all'; recoverAll = true;
case 'block'; recoverAll = false;
otherwise; error('unknown flag');
end;
  
Naug  = TheJournal.Index.Naug;
Xa_i  = TheJournal.Index.Xa_i;
Xf_ii = TheJournal.Index.Xf_ii;
Nf    = length(Xf_ii);
Ne    = length(Xf_ii{1});

Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
%p = symmmd(Lambda);
p = symamd(Lambda);
Lchol = chol(Lambda(p,p));

if recoverAll;
  Sigma = zeros(Naug,Naug);
else;
  Sigma = spalloc(Naug,Naug,Nf*Ne^2);
end;

for fn=1:Nf;
  disp(fn);
  % feature indices
  Xfi_i = Xf_ii{fn};
  
  % basis vectors of delayed state
  E = spalloc(Naug,Ne,Ne);
  E(Xfi_i,:) = speye(Ne);
  
  % covariance column
  SigmaCol = zeros(Naug,Ne);
  SigmaCol(p,:) = full( Lchol \ (Lchol' \ E(p,:)) );
  
  if recoverAll
    % store covariance column
    Sigma(:,Xfi_i) = SigmaCol;
  else;
    % extract block covariance
    Sigma(Xfi_i,Xfi_i) = symmetrize(SigmaCol(Xfi_i,:));
  end;
end;
