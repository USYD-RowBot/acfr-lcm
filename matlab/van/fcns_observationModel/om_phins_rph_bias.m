function [zpredict,z_fix,R_fix,Hv] = om_phins_rph_bias(Xv,index_t,z_raw)
%INPUTS:
%  Xv is the vehicle state vector assumed to contain at 
%  least the following elements, (not necessarily in the order shown)
%  Xv = [x y z r p h]'
%  index_t is a structure of state indices
%  z_raw raw attitude measurement in sensor frame
%
%  zpredict is measurement
%  Hv observation matrix
%  z_fix unwrapped attitude measurement
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    02-27-2004      rme         Created from om_xbow.m
%    03-29-2004      rme         Incorporated nonlinear observation model.
%    04-12-2004      rme         Updated to use index_t structure.
%    04-21-2004      rme         Created from om_phins_rph.m
%    11-26-2004      rme         Reordered output args to make Hv last.
  
Xp_i = index_t.Xp_i;             % pose variable index
rph_bias_i = index_t.rph_bias_i; % phins bias variable index

% vehicle pose in local-level frame
x_lv = Xv(Xp_i);

% current estimate of static sensor to vehicle transform
x_vs = [0;0;0;Xv(rph_bias_i)];

% predicted phins sensor pose in local-level frame based
% upon current vehicle state
[x_ls,J_plus] = head2tail(x_lv,x_vs);

% predicted measurement
% z = [r_sensor_frame
%      p_sensor_frame
%      h_sensor_frame]
zpredict = x_ls(4:6);

% measurement Jacobian
% Hv = [d(r_sensor_frame)/dXv
%       d(p_sensor_frame)/dXv
%       d(h_sensor_frame)/dXv]
Hv = spalloc(3,index_t.Nv,18);
Hv(:,[Xp_i,rph_bias_i]) = J_plus(4:6,[1:6,10:12]);

% unwrap the raw phins attitude measurment so that it is consistent
% with the predicted vehicle measurment
z_fix = funwrap([zpredict,z_raw],2);
z_fix = z_fix(:,2);

R_fix = R_raw;
