function meas_t = add_octans_sensor_meas(nav_t,mindex_t,TheConfig)
%function meas_t = add_octans_sensor_meas(nav_t,mindex_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written.
%    04-08-2004      rme         Modified to use mindex_t structure
%    04-12-2004      rme         Updated to work with TheConfig.use function handle
%    04-21-2004      rme         Added phins bias
%    05-17-2004      rme         Simplified to reduce code maintenance
%    09-01-2004      rme         Modified to use TheConfig.ObservationModel field  
%    10-28-2004      rme         Major code reorganization.

%raw phins attitude measurement
rph_raw = [nav_t.PHINS.roll(mindex_t.PHINS); ...    % roll
	   nav_t.PHINS.pitch(mindex_t.PHINS); ...   % pitch
	   nav_t.PHINS.heading(mindex_t.PHINS)];    % heading
Cov_rph_raw = diag([TheConfig.SensorNoise.PHINS.roll; ...
		    TheConfig.SensorNoise.PHINS.pitch; ...
		    TheConfig.SensorNoise.PHINS.heading].^2);

% stuff the measurement and covariance
meas_t.omfhandle = TheConfig.ObservationModel.PHINS;
meas_t.omfhandleString = func2str(meas_t.omfhandle);
meas_t.isaDelayedStateMeas  = false;
meas_t.z        = rph_raw;
meas_t.R        = Cov_rph_raw;
meas_t.varargin = {TheConfig.SensorXform.PHINS.x_vs}; % sensor pose w.r.t. to vehicle frame
