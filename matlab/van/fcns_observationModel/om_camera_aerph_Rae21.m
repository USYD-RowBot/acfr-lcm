function [z_predict,z_fix,R_fix,Haug] = ...
    om_camera_aerph_Rae21(mu,index_t,z_raw,R_raw,Xfi_i,Xfj_i,Xvc_arg,estimateCameraXform)
%function [z_predict,Haug,z_fix,R_fix] = ...
%    om_camera_aerph_Rae21(mu,index_t,z_raw,R_raw,Xfi_i,Xfj_i,Xvc_arg,estimateCameraXform)  
%
% relative pose of camera 1 w.r.t. camera 2 frame
%
%INPUTS:
%  mu      is the augmented state vector mean
%  index_t is the state index data structure
%  z_raw   is the raw measurement
%  R_raw   is the raw measurement covariance
%  Xfi_i   is the index of camera state i
%  Xfj_i   is the index of camera state j
%  Xvc_arg can be either be x_vc or x_vc_i
%  estimateCameraXform is a binary flag
%
%  If Xvc_arg = x_vc_i, then the camera to vehicle tranform
%  is pulled from mu.  Otherwise Xvc_arg = x_vc is the known
%  static camera to vehicle sensor transform.
%
%OUTPUT:
%  z_predict predicted 5 DOF measurement [az,el,r,p,h]'
%  Haug analytical Jacobian w.r.t. Xfi_i Xfj_i (x_vc_i)
%  z_fix angular unwrapped raw measurment
%  R_fix associated covariance matrix
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-06-2004      rme         Created from om_camera_Rt21.m
%    04-13-2004      rme         Modified to use relative_sensor_pose.m
%    11-03-2004      rme         Modified input argument list to compute z_fix,R_fix
%    11-26-2004      rme         Reorded output args to make Haug last.

% shorthand index
Np   = index_t.Np;   % number of pose elements
Naug = index_t.Naug; % number of augmented state elements

% check Xvc_arg to see if it is an index vector or a 6 DOF pose vector.
if estimateCameraXform % it's an index vector
  x_vc_i = Xvc_arg;
  x_vc  = mu(x_vc_i);
else % it's a 6 DOF pose vector
  x_vc_i = [];
  x_vc  = Xvc_arg;
end

%======================================================
% extract delayed state vehicle poses from the augmented
% state vector.  these are the vehicle poses
% at the time the images were taken.
%======================================================
x_lvi = mu(Xfi_i);
x_lvj = mu(Xfj_i);

%======================================================
% calculate relative pose of camera i w.r.t camera j.
%======================================================
[x_cjci,J_lvj_lvi_vc] = relative_sensor_pose(x_lvj,x_lvi,x_vc);

% decompose into relative pose parameters
t   = x_cjci(1:3);
rph = x_cjci(4:6);

%======================================================
% camera 5 DOF observation model
%======================================================
% baseline direction and associated Jacobian
[b,Jb] = trans2dm(t);

% predicted measurement z_ji
z_predict = [b(1:2); rph]; % [az,el,r,p,h]

%======================================================
% calculate the Jacobian associated with the
% predicted measurement
%======================================================
% allocate sparse augmented state Jacobian
Haug = spalloc(5,Naug,5*Np*3);
  
% Jacobian of measurement w.r.t. relative camera pose
% i.e. d(z_ji)/d(x_cjci)
J_cjci = [Jb(1:2,:), zeros(2,3);
	  zeros(3),  eye(3)];

% fill in the augmented state Jacobian via chain-rule
Haug(:,Xfi_i) = J_cjci * J_lvj_lvi_vc(:,7:12);
Haug(:,Xfj_i) = J_cjci * J_lvj_lvi_vc(:,1:6);
if ~isempty(x_vc_i)
  % Jacobian w.r.t. camera to vehicle transform
  Haug(:,x_vc_i) = J_cjci * J_lvj_lvi_vc(:,13:18);
end

%=======================================================
% unwrap raw measurement so that it is consistent
% with the predicted state vector measurement
%=======================================================
z_fix = funwrap([z_predict,z_raw],2);
z_fix = z_fix(:,2);
R_fix = R_raw;
