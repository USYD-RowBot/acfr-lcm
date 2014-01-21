function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_ekf(fni,fnj,TheConfig);
%function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_ekf(fni,fnj,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-05-2004      rme         Created and written.
%    01-12-2005      rme         Fixed ordering of Sigma elements to be
%                                consistent with order of return args.

global TheJournal;

% shorthand index
Np    = TheJournal.Index.Np;                % number of pose elements
Xp_i  = TheJournal.Index.Xp_i;              % pose element index
Xfi_i = TheJournal.Index.Xf_ii{fni}(Xp_i);  % delayed state pose index at time t1
Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);  % delayed state pose index at time t2

% extract vehicle poses
x_lvi = TheJournal.Ekf.mu(Xfi_i);           % 6 DOF vehicle state at time t1
x_lvj = TheJournal.Ekf.mu(Xfj_i);           % 6 DOF vehicle state at time t2

% extract delayed state poses from the augmented state vector corresponding
% to the vehicle pose at the time the image was taken.  
% also check if the camera to vehicle transform x_vc is a parameter in the 
% state vector and if so extract it as well.
%---------------------------------------------------------------------------
if TheConfig.Estimator.estimateCameraXform;
  % 6 DOF camera to vehicle transform is static but unknown,
  % it is being estimated within the state vector
  x_vc_i = TheJournal.Index.x_vc_i; % state index of camera to vehicle transform
  x_vc   = TheJournal.Ekf.mu(x_vc_i);  
  % joint covariance matrix associated with pose elements
  Sigma = TheJournal.Ekf.Sigma([Xfi_i,Xfj_i,x_vc_i],[Xfi_i,Xfj_i,x_vc_i]);
else;
  % 6 DOF camera to vehicle transform is static and known
  x_vc = TheConfig.SensorXform.PXF.x_vs;
  % joint covariance matrix associated with pose elements and *known* static x_vc transform
  Sigma = blkdiag(TheJournal.Ekf.Sigma([Xfi_i,Xfj_i],[Xfi_i,Xfj_i]),zeros(Np));
end;
