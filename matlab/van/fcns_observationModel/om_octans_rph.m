function [zpredict,z_fix,R_fix,Hv] = om_octans_rph(Xv,index_t,z_raw,R_raw,x_vs)
%INPUTS:
%  Xv is the vehicle state vector assumed to contain at 
%  least the following elements, (not necessarily in the order shown)
%  Xv = [x y z r p h]'
%  index_t is a structure of state indices
%  z_raw raw attitude measurement in sensor frame
%  x_vs [6 x 1] pose vector of sensor w.r.t. vehicle frame
%
%  zpredict is measurement
%  Hv observation matrix
%  z_fix unwrapped attitude measurement
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-02-2004      rme         Created from om_phins_rph.m
%    11-26-2004      rme         Reordered output args to make Hv last.

% pose variable index
Xp_i = index_t.Xp_i;

% vehicle pose in local-level frame
x_lv = Xv(Xp_i);

% xform from local-level to octans reference frame
x_rl = zeros(6,1);

% predicted octans attitude measurement w.r.t. sensor reference frame
[x_ls,Jls_plus] = head2tail(x_lv,x_vs);
[x_rs,Jrs_plus] = head2tail(x_rl,x_ls);
% complete Jacobian d(x_rs)/d(x_lv) = d(x_rs)/d(x_ls) * d(x_ls)/d(x_lv)
J_plus = Jrs_plus(:,7:12) * Jls_plus(:,1:6);

% predicted measurement
% z = [r_sensor_frame
%      p_sensor_frame
%      h_sensor_frame]
zpredict = x_rs(4:6);

% measurement Jacobian
% Hv = [d(r_sensor_frame)/dXv
%       d(p_sensor_frame)/dXv
%       d(h_sensor_frame)/dXv]
Hv = spalloc(3,index_t.Nv,18);
Hv(:,Xp_i) = J_plus(4:6,:);

% unwrap the raw octans attitude measurment so that it is consistent
% with the predicted vehicle measurment
z_fix = funwrap([zpredict,z_raw],2);
z_fix = z_fix(:,2);
R_fix = R_raw;
