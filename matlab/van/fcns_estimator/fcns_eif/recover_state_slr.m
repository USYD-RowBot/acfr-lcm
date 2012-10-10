function varargout = recover_state_slr(nIterations,nGroupSize);
%function [residual,resnorm] = recover_state_slr(nIterations,nGroupSize);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-18-2004      rme         Created and written.
%    11-27-2004      rme         Modified to work on groups of block states.

global TheJournal;

% defaults
if ~exist('nIterations','var') || isempty(nIterations); nIterations = 1; end; % default 1 iteration
if ~exist('nGroupSize','var') || isempty(nGroupSize); nGroupSize = 24; end; % default 24 states in a group

% shorthand index
Nf    = TheJournal.Index.Nf;
Xa_i  = TheJournal.Index.Xa_i;
Xf_ii = TheJournal.Index.Xf_ii;

% pointers into TheJournal
eta    = TheJournal.Eif.eta(Xa_i);
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);

nGroups = max(1, round((Nf+1)/nGroupSize) );
% perform nIterations of Gauss-Siedel
for k=1:nIterations;
  for n=Nf+1:-nGroupSize:1;
    fn = n:-1:max(1,n-nGroupSize+1);  % feature numbers in group
    ii = [Xf_ii{fn}];         % indices of features in group
    TheJournal.Eif.mu(ii) = TheJournal.Eif.mu(ii) + ...
	spdmldivide( Lambda(ii,ii), eta(ii)-Lambda(ii,:)*TheJournal.Eif.mu(Xa_i) );
  end;
end;

if nargout >= 1;
  % compute the residual
  residual = eta - Lambda*mu;
  varargout{1} = residual;
end;
if nargout == 2;
  % compute the normalized residual L2 norm
  eta_norm = norm(eta);
  resnorm = norm(residual)/eta_norm;
  varargout{2} = resnorm;
end
