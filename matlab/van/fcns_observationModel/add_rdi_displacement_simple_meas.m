function meas_t = add_rdi_displacement_simple_meas(dt,nav_t,mindex_t,TheConfig)
%function meas_t = add_rdi_displacement_simple_meas(dt,nav_t,mindex_t,TheConfig)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written. 
%    04-08-2004      rme         Modified to use mindex_t structure

fprintf('************************************************************\n');
fprintf('Doppler dropout of dt=%g @ t=%g,\n',dt,TheJournal.t);
fprintf('using displacement measurement.\n');
fprintf('************************************************************\n');

Rlw = transpose(TheConfig.SensorXform.Rwl); % world frame to local-level frame

% absolute position at time t1 in world frame
xyz1_w = [nav_t.RDI.nx(mindex_t.RDI-1); ...
	  nav_t.RDI.ny(mindex_t.RDI-1); ...
	  nav_t.RDI.nz(mindex_t.RDI-1)];
% absolute position at time t2 in world frame
xyz2_w = [nav_t.RDI.nx(mindex_t.RDI); ...
	  nav_t.RDI.ny(mindex_t.RDI); ...
	  nav_t.RDI.nz(mindex_t.RDI)];

meas_t.omfhandle = @om_rdi_displacement;
meas_t.isaDelayedStateMeas  = true;
meas_t.z        = Rlw*(xyz2_w - xyz1_w); % displacement measured in local-level frame

uvw = [nav_t.RDI.u_cooked(mindex_t.RDI-1); ... % body frame velocities
       nav_t.RDI.v_cooked(mindex_t.RDI-1); ...
       nav_t.RDI.w_cooked(mindex_t.RDI-1)];
rph = [nav_t.RDI.roll_cooked(mindex_t.RDI-1); ... % vehicle roll,pitch,heading
       nav_t.RDI.pitch_cooked(mindex_t.RDI-1); ...
       nav_t.RDI.heading_cooked(mindex_t.RDI-1)];
Rlv = rotxyz(rph); % = rotz'*roty'*rotx'       % vehicle to local-level

% note, since i'm missing the *actual* RDI measurements during this dropout
% period, i'm only able to calculate the Jacobian at the start and endpoints
% of this period.  i'm approximating the total displacement covariance by
% using the start Jacobian throughout the whole summation.
J = zeros(3,6);
J(1:3,1:3) = Rlv; % deriv w.r.t. uvw
J(:,4) = rotz(rph(3))'*roty(rph(2))'*drotx(rph(1))'*uvw; % deriv w.r.t. r
J(:,5) = rotz(rph(3))'*droty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. p
J(:,6) = drotz(rph(3))'*roty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. h

% individual displacement covariance where 0.25s is the integration period
% note that 0.25s is the typical sampling period of the RDI
Cov_uvw = diag([TheConfig.SensorNoise.RDI.u; ...
		TheConfig.SensorNoise.RDI.v; ...
		TheConfig.SensorNoise.RDI.w].^2);
Cov_rph = diag([TheConfig.SensorNoise.RDI.roll; ...
		TheConfig.SensorNoise.RDI.pitch; ...
		TheConfig.SensorNoise.RDI.heading].^2);
lambda = [Cov_uvw,  zeros(3); % measurement covariance
	  zeros(3), Cov_rph];
lambda_i = 0.25^2 * J*lambda*J';

% estimated number of missed RDI measurments
k = ceil(dt/0.25);

% measurement covariance
meas_t.R = k*lambda_i; % approximate covariance of *total displacment*
		       % based upon a simple summation of individual covariances
		       % assumming independence
meas_t.varargin = {};

