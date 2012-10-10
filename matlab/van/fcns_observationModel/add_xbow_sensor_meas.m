function meas_t = add_xbow_sensor_meas(nav_t,mindex_t,TheConfig)
%function meas_t = add_xbow_sensor_meas(nav_t,mindex_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written.
%    04-08-2004      rme         Modified to use mindex_t structure
%    04-12-2004      rme         Updated to work with TheConfig.use function handle
%    05-17-2004      rme         Simplified to reduce code maintenance
%    09-01-2004      rme         Modified to use TheConfig.ObservationModel field
%    10-28-2004      rme         Major code reorganization.

% raw xbow attitude rate measurement
rphdot_raw = [nav_t.XBOW.rr(mindex_t.XBOW); ... % roll rate
	      nav_t.XBOW.pr(mindex_t.XBOW); ... % pitch rate
	      nav_t.XBOW.hr(mindex_t.XBOW)];    % heading rate
Cov_rphdot_raw = diag([TheConfig.SensorNoise.XBOW.rr; ...
		       TheConfig.SensorNoise.XBOW.pr; ...
		       TheConfig.SensorNoise.XBOW.hr].^2);

% stuff the measurement and covariance
meas_t.omfhandle = TheConfig.ObservationModel.XBOW;
meas_t.omfhandleString = func2str(meas_t.omfhandle);
meas_t.isaDelayedStateMeas  = false;
meas_t.z        = rphdot_raw;
meas_t.R        = Cov_rphdot_raw;
meas_t.varargin = {TheConfig.SensorXform.XBOW.Rvs};
