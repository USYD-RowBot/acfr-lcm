function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni,fnj,TheConfig);
%function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni,fnj,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-12-2004      rme         Created and written.
%    11-14-2004      rme         Modified to key off of TheConfig.Estimator.inferenceEif

global TheJournal;

% extract pose information from our state estimate
if TheConfig.Estimator.inferenceEif;
  [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_eif(fni,fnj,TheConfig);
else;
  [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_ekf(fni,fnj,TheConfig);
end;
