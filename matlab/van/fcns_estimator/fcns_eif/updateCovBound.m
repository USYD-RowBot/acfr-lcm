function [] = updateCovBound(meas_t,TheConfig);
%function [] = updateCovBound(meas_t,TheConfig);  
%
% function accepts a single meas_t structure, therefore call in a "for-loop"
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2005-01-19      rme         Created and written from update_aug_iekf.m

global TheJournal;


% evaluate the predicted measurement and the Jacobian of the observation
% model.  allow passing of optional arguments to observation function
%--------------------------------------------
% measurement prediction
[z_predict,z_fix,R_fix,H] = feval(meas_t.omfhandle, TheJournal.Eif.mu, TheJournal.Index, ...
				  meas_t.z, meas_t.R, meas_t.Xfi_i, meas_t.Xfj_i, meas_t.varargin{:});
    
if isempty(H);
  % shorthand index
  Nv   = TheJournal.Index.Nv;   % size of vehicle state
  Naug = TheJournal.Index.Naug; % size of augmented state
  Xa_i = TheJournal.Index.Xa_i; % "all" state index

  % allocate a sparse Jacobian pattern with m*Nv*2 non-zero elements
  % m is the size of the measurement vector
  % Nv is the number of variables associated with a delayed state
  % 2 is the number of delayed states involved in a relative pose measurement
  m = length(meas_t.z);
  Jpattern = spalloc(m,Naug,m*Nv*2);
  Jpattern(:,[meas_t.Xfi_i, meas_t.Xfj_i]) = 1;
    
  % numerically evaluate the sparse Jacobian of the observation model w.r.t. the state vector
  fprintf('==>%s: Computing numerical Jacobian for %s\n',mfilename,meas_t(jj).omfhandleString);
  H = numerical_jacobian(meas_t.omfhandle, TheJournal.Eif.mu, z_predict, Jpattern, ...
			 TheJournal.Index, meas_t.z, meas_t.R, ...
			 meas_t.Xfi_i, meas_t.Xfj_i, meas_t.varargin{:});
end;

% project jacobian to 2-state size
%-------------------------------------------
H = H(:,[meas_t.Xfi_i,meas_t.Xfj_i]);

% Sigma_joint for conservative Kalman update
%--------------------------------------------
Sigma_joint = [TheJournal.Eif.SigmaBound{meas_t.fni},    TheJournal.Eif.SigmaCol(meas_t.Xfi_i,:); ...
	       TheJournal.Eif.SigmaCol(meas_t.Xfi_i,:)', TheJournal.Eif.SigmaCol(meas_t.Xfj_i,:)];

% innovation covariance
%--------------------------------------------
S  = R_fix + spdproduct(Sigma_joint,H');
  
% Kalman gain
%--------------------------------------------
K = Sigma_joint*H'*spdinverse(S);
K = sparse(K);

% covariance update (based on Joseph form)
%--------------------------------------------
I = eye(size(Sigma_joint));
IminusKH = (I-K*H);
Sigma_joint = spdproduct(Sigma_joint,IminusKH') + spdproduct(R_fix,K');


% retain updated bound
%--------------------------------------------
TheJournal.Eif.SigmaBound{meas_t.fni} = symmetrize(Sigma_joint(1:6,1:6));
