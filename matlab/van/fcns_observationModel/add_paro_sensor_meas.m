function meas_t = add_paro_sensor_meas(nav_t,mindex_t,TheConfig)
%function meas_t = add_paro_sensor_meas(nav_t,mindex_t,TheConfig)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written.
%    04-08-2004      rme         Modified to use mindex_t structure
%    09-01-2004      rme         Modified to use TheConfig.ObservationModel field
%    10-28-2004      rme         Major code reorganization.
  
meas_t.omfhandle = TheConfig.ObservationModel.PARO;
meas_t.omfhandleString = func2str(meas_t.omfhandle);
meas_t.isaDelayedStateMeas  = false;                         % delayed state meas?
meas_t.z        = nav_t.PARO.depth(mindex_t.PARO);           % raw sensor depth
meas_t.R        = TheConfig.SensorNoise.PARO.depth^2;         % measurement covariance
meas_t.varargin = {TheConfig.SensorXform.PARO.x_vs};          % sensor pose w.r.t. to vehicle frame
