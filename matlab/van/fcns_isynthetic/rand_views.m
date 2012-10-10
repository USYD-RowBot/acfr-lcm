function [Rt,uu_noise,vv_noise,uu_true,vv_true] = rand_views(Nviews,dist,spread,noise,X,Y,Z,K)
%function [Rt,uu_noise,vv_noise,uu_true,vv_true] = rand_views(Nviews,dist,spread,noise,X,Y,Z,K)
% generate Nviews random camera positions, taking care to ensure that all of
% the cameras are on the same side of the scene and are all looking at the
% scene.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created and written.

Nf = length(Z);

Rcw = [1  0  0;
       0 -1  0;
       0  0 -1];
uu_true = [];
vv_true = [];
uu_noise = [];
vv_noise = [];
Rt = [];
for ii=1:Nviews
  % orientation of camera w.r.t. scene frame
  rph = diag([5,5,180]*pi/180)*(2*rand(3,1)-1);
  R = rotxyz(rph)*Rcw;
  % orientation of scene frame w.r.t. camera
  R_T = R';
  
  % position of camera center w.r.t. scene frame
  c = [spread*(2*rand(2,1)-1); dist+randn];
  % make sure cameras stay on same side of scene
  while c(3) < max(Z)
    c(3) = dist+randn;
  end
  % position of scene frame w.r.t. camera
  t = -R_T*c;
  
  % camera projection matrix
  P = K*[R_T t];
  
  % projection into image plane
  [u,v] = pinhole_project(P,X,Y,Z);

  % store motion and image points
  Rt = [Rt, R_T,t];
  uu_true = [uu_true, u];
  vv_true = [vv_true, v];
  uu_noise = [uu_noise, u+noise*randn(Nf,1)];
  vv_noise = [vv_noise, v+noise*randn(Nf,1)];
end
