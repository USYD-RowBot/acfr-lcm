function meas_t = add_rdi_sensor_meas(nav_t,mindex_t,TheConfig)
%function meas_t = add_rdi_sensor_meas(nav_t,mindex_t,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written. 
%    04-08-2004      rme         Modified to use mindex_t structure
%    04-12-2004      rme         Updated to work with TheConfig.use function handle
%    04-23-2004      rme         Simplified to reduce code maintenance
%    09-01-2004      rme         Modified to use TheConfig.ObservationModel field
%    09-01-2004      rme         Modified to use RDI velocity error term
%                                if available
%    10-28-2004      rme         Major code reorganization.

% raw velocity measurement
uvw_raw = [nav_t.RDI.u(mindex_t.RDI); ...     % sensor frame x velocity
	   nav_t.RDI.v(mindex_t.RDI); ...     % sensor frame y velocity
	   nav_t.RDI.w(mindex_t.RDI)];        % sensor frame z velocity
if isfield(nav_t.RDI,'e') ...
      && (nav_t.RDI.e(mindex_t.RDI) ~= 0) % 3-beam soln can't compute error velocity term
  % use the RDI measured error velocity as the 1-sigma standard deviation
  Cov_uvw_raw = diag(nav_t.RDI.e(mindex_t.RDI)^2*[1,1,1]);
else
  Cov_uvw_raw = diag([TheConfig.SensorNoise.RDI.u; ...
		      TheConfig.SensorNoise.RDI.v; ...
		      TheConfig.SensorNoise.RDI.w].^2);
end

% raw attitude measurement
rph_raw = [nav_t.RDI.roll(mindex_t.RDI); ...  % Euler roll
	   nav_t.RDI.pitch(mindex_t.RDI); ... % Euler pitch
	   nav_t.RDI.heading(mindex_t.RDI)];  % Euler heading
Cov_rph_raw = diag([TheConfig.SensorNoise.RDI.roll; ...
		    TheConfig.SensorNoise.RDI.pitch; ...
		    TheConfig.SensorNoise.RDI.heading].^2);

% stuff sensor measurements
meas_t.omfhandle = TheConfig.ObservationModel.RDI;
meas_t.omfhandleString = func2str(meas_t.omfhandle);
meas_t.isaDelayedStateMeas  = false;
meas_t.z        = [uvw_raw; rph_raw];
meas_t.R        = [Cov_uvw_raw, zeros(3);
		   zeros(3),    Cov_rph_raw];
meas_t.varargin = {TheConfig.SensorXform.RDI.Rvs,TheConfig.SensorXform.RDI.x_vs};
