function [z_ji,J] = om_camera_xyzrph_Rt21(Xaug,Xfi_ind,Xfj_ind,Rvc,tvc_v)
% relative pose of camera 1 w.r.t. camera 2 frame
%
%INPUTS:
%  Xaug is the augmented state vector
%  Xfi_ind is the index of camera state i
%  Xfj_ind is the index of camera state j
%
%OUTPUT:
%  z_ji predicted 5 DOF measurement
%  J  analytical Jacobian J = [Jxi Jxj]
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-27-2003      rme         Created.
%    10-14-2003      rme         Added calculation of analytical Jacobian.
%    11-15-2203      rme         Switched to SSC head2tail and tail2tail
%                                coordinate frame relationship
%                                functions.  Final Jacobian result is
%                                obtained by compounding intermediate
%                                results based upon chain-rule.
  
%======================================================
% extract delayed state vehicle pose from the augmented
% state vector note this corresponds to the vehicle pose
% at the time the image was taken
%======================================================
Xlvi = Xaug(Xfi_ind);
Xlvj = Xaug(Xfj_ind);

%======================================================
% calculate relative pose of camera i w.r.t camera j.
%======================================================
% static homogenous xform from camera to vehicle
Xvc = [tvc_v; rot2rph(Rvc)];

% pose of camera i in local-level
[Xlci,Jlci] = head2tail(Xlvi,Xvc);

% pose of camera j in local-level
[Xlcj,Jlcj] = head2tail(Xlvj,Xvc);

% relative pose of camera i w.r.t. camera j
[Xcjci,Jcjci] = tail2tail(Xlcj,Xlci);
% Jacobian of Xcjci w.r.t. Xlvj Xlvi
J_Xcjci = [Jcjci(:,1:6) *Jlcj(:,1:6), Jcjci(:,7:12)*Jlci(:,1:6)];

% decompose into relative pose parameters
t = Xcjci(1:3);
rph = Xcjci(4:6);

%======================================================
% camera 5 DOF observation model
%======================================================
% baseline direction and associated Jacobian
[b,Jb] = unitize(t);

% predicted measurement
z_ji = [b; rph];

%======================================================
% calculate the Jacobian associated with the
% predicted measurement
%======================================================
if nargout == 2
  J = [Jb*J_Xcjci(1:3,:);
          J_Xcjci(4:6,:)];

  % the output Jacobian assumes w.r.t. (Xlvi,Xlvj) not (Xlvj,Xlvi)
  % therefore swap left and right halves
  J = J(:,[[7:12],[1:6]]);
end
