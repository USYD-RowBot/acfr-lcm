function [eta,Lambda] = augstate_pwc_eif(eta,Lambda,index_t,F,B,u,Q)
%function [eta,Lambda] = augstate_pwc_eif(eta,Lambda,index_t,F,B,u,Q)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created and written.
%    10-14-2004      rme         Added additional comments.
  
% shorthand index into augmented state vector
Xv_i = index_t.Xv_i; % vehicle state index
Xf_i = index_t.Xf_i; % feature state index (all)

% extract some useful terms
%mu_x  = mu_est(Xv_i);
%mu_M  = mu_est(Xf_i);
eta_x = eta(Xv_i);              % robot(t)
eta_M = eta(Xf_i);              % map
Lambda_xx = Lambda(Xv_i,Xv_i);
Lambda_xM = Lambda(Xv_i,Xf_i);
Lambda_MM = Lambda(Xf_i,Xf_i);

% precompute some useful terms
Q_inv  = symmetrize(Q^-1);
Q_invF = Q_inv*F;
Omega  = symmetrize(Lambda_xx + F'*Q_inv*F);
Zeros  = spalloc(length(Xv_i),length(Xf_i),0);

% ~~~~~~~~~~~~~~ state augmentation ~~~~~~~~~~~~~~~~~~~~~~~
% reorder the previous state to the end of the state vector
% and prepend with the current time projected state
%----------------------------------------------------------
% information vector
eta =  [ Q_inv*B*u; ...          % x(t+1)
	 eta_M;     ...          % M
	 eta_x - F'*Q_inv*B*u ]; % x(t)
% information matrix
Lambda = sparse([ Q_inv,   Zeros,     -Q_invF; ...      % x(t+1)
		  Zeros',  Lambda_MM,  Lambda_xM'; ...  % M
		 -Q_invF', Lambda_xM,  Omega ]);        % x(t)

% update mean estimate
%mu_est = [F*mu_x + B*u; ...
%	    mu_M; ...
%	    mu_x ]; 
