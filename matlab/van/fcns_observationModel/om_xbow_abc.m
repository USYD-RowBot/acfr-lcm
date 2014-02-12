function [zpredict,z_fix,R_fix,Hv] = om_xbow_abc(Xv,index_t,z_raw,R_raw,Rvs)
%INPUTS:
%  Xv is the vehicle state vector assumed to contain the following elements
%  Xv = [a b c]
%  index_t is a structure of state indices
%  z_raw raw attitude rate measurement in sensor frame
%  Rvs rotation matrix from sensor to vehicle
%
%  zpredict is measurement
%  Hv observation matrix
%  z_fix same as z_raw
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-28-2003      rme         Created and written.
%    03-29-2004      rme         Modified observation model to predict
%                                sensor measurement instead of putting
%                                sessor in vehicle frame.
%    04-12-2004      rme         Updated to use index_t structure.
%    11-26-2004      rme         Reordered output args to make Hv last.

Rsv = Rvs'; % rotation matrix from vehicle to sensor  

% attitude rate variable index
abc_i = index_t.abc_i;

% transform vehicle frame rate measurements to xbow sensor frame
% to get predicted sensor measurement
% zpredict = a_sensor_frame
%          = b_sensor_frame
%          = c_sensor_frame
zpredict = Rsv * Xv(abc_i);

% measurement Jacobian
Hv = spalloc(3,index_t.Nv,9);
Hv(:,abc_i) = Rsv;
  
% no need to modify the raw rate measurement
z_fix = z_raw;
R_fix = R_raw;
