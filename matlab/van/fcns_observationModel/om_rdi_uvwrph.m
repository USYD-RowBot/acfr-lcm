function [zpredict,z_fix,R_fix,Hv] = om_rdi_uvwrph(Xv,index_t,z_raw,R_raw,Rvs,x_vs)
%INPUTS:
%  Xv is the vehicle state vector assumed to contain as least the following elements
%  (not necessarily in the order shown)
%  Xv = [x y z r p h u v w a b c]'
%  index_t is a structure of state indices
%  z_raw is the raw sensor measurement in the sensor frame
%  Rvs is the sensor to vehicle rotation matrix
%  x_vs [6 x 1] pose vector of sensor w.r.t. vehicle frame
%
%  zpredict is measurment
%  Hv observation matrix
%  z_fix sensor frame velocities and unwrapped attitude measurement    
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-28-2003      rme         Created and written.
%    04-12-2004      rme         Updated to use index_t structure.
%    04-23-2004      rme         Added omega cross R term to uvw
%                                observation model.
%    11-26-2004      rme         Reordered output args to make Hv last.

% rotation matrix from vehicle to sensor frame
Rsv = Rvs';
% vector from vehicle to sensor in vehicle frame 
tvs_v = x_vs(1:3);

% shorthand of state vector indices
Xp_i  = index_t.Xp_i;  % pose variable index
uvw_i = index_t.uvw_i; % vehicle frame velocity
rph_i = index_t.rph_i; % vehicle frame orientation
abc_i = index_t.abc_i; % vehicle frame angular rates

% shorthand of state vector values
x_lv = Xv(Xp_i);  % vehicle pose in local-level
uvw  = Xv(uvw_i); % body frame velocities
rph  = Xv(rph_i); % euler angles
abc  = Xv(abc_i); % body frame angular rates

% predicted attitude measurement of sensor frame
[x_ls,J_plus] = head2tail(x_lv,x_vs);
rph_s_predict = x_ls(4:6);

% predicted velocity measurement in sensor frame
uvw_s_predict = Rsv * [uvw - skewsym(tvs_v)*abc];

% predicted measurement
% z = [u_sensor_frame
%      v_sensor_frame
%      w_sensor_frame
%      r_sensor_frame
%      p_sensor_frame
%      h_sensor_frame]
zpredict = [uvw_s_predict;
	    rph_s_predict];

% measurement Jacobian
% Hv = [d(u_sensor_frame)/dXv
%       d(v_sensor_frame)/dXv
%       d(w_sensor_frame)/dXv
%       d(r_sensor_frame)/dXv
%       d(p_sensor_frame)/dXv
%       d(h_sensor_frame)/dXv]
Hv = spalloc(6,index_t.Nv,27);
Hv(1:3,uvw_i) =  Rsv;
Hv(1:3,abc_i) = -Rsv*skewsym(tvs_v);
Hv(4:6,Xp_i)  =  J_plus(4:6,1:6);

% unwrap the raw rdi attitude measurment so that it is consistent
% with the predicted vehicle measurement
rph_raw = z_raw(4:6);
rph_fix = funwrap([rph_s_predict,rph_raw],2);
rph_fix = rph_fix(:,2);

z_fix = z_raw;
z_fix(4:6) = rph_fix;

R_fix = R_raw;
