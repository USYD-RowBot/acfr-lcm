function meas_t = add_rdi_displacement_full_meas(nav_t,dt,TheConfig)
%function meas_t = add_rdi_displacement_full_meas(nav_t,dt,TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written. 

fprintf('************************************************************\n');
fprintf('Doppler dropout of dt=%g @ t=%g,\n',dt,TheJournal.t);
fprintf('using displacement measurement.\n');
fprintf('************************************************************\n');

Rlw = transpose(TheConfig.SensorXform.Rwl); % world frame to local-level frame

xyz1_w = [nav_t.RDI.nx(nav_t.RDI.mindex-1); ...
	  nav_t.RDI.ny(nav_t.RDI.mindex-1); ...
	  nav_t.RDI.nz(nav_t.RDI.mindex-1)];
xyz2_w = [nav_t.RDI.nx(nav_t.RDI.mindex); ...
	  nav_t.RDI.ny(nav_t.RDI.mindex); ...
	  nav_t.RDI.nz(nav_t.RDI.mindex)];

      
meas_t.omfhandle = @om_rdi_displacement;
meas_t.isaDelayedStateMeas  = true;
meas_t.z        = [Rlw*(xyz2_w - xyz1_w); ... % displacement measured in local-level frame
		   nav_t.RDI.u(nav_t.RDI.mindex); ...
		   nav_t.RDI.v(nav_t.RDI.mindex); ...
		   nav_t.RDI.w(nav_t.RDI.mindex); ...
		   nav_t.RDI.roll(nav_t.RDI.mindex); ...
		   nav_t.RDI.pitch(nav_t.RDI.mindex); ...
		   nav_t.RDI.heading(nav_t.RDI.mindex)];

uvw = meas_t.z(4:6);
rph = meas_t.z(7:9);
Rlv = rotxyz(rph);

% note, since i'm missing the actual RDI measurements during this period,
% i'm able to calculate the actual Jacobian only at *this point* in time.  i'm
% approximating the total displacement covariance by using this Jacobian
% throughout the whole summation.
J = zeros(3,6);
J(1:3,1:3) = Rlv; % deriv w.r.t. uvw
J(:,4) = rotz(rph(3))'*roty(rph(2))'*drotx(rph(1))'*uvw; % deriv w.r.t. r
J(:,5) = rotz(rph(3))'*droty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. p
J(:,6) = drotz(rph(3))'*roty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. h

% individual displacement covariance where 0.25 is the integration period
Cov_uvw = diag([nav_t.RDI.sigma.u; ...
		nav_t.RDI.sigma.v; ...
		nav_t.RDI.sigma.w].^2);
Cov_rph = diag([nav_t.RDI.sigma.roll; ...
		nav_t.RDI.sigma.pitch; ...
		nav_t.RDI.sigma.heading].^2);
lambda = [Cov_uvw;  zeros(3); % measurement covariance
	  zeros(3); Cov_rph];
lambda_i = J*lambda*J';

% estimated number of missed RDI measurments
k = ceil(dt/0.25);

% measurment covariance
meas_t.R = zeros(9);
meas_t.R(1:3,1:3) = k*lambda_i; % approximate covariance of *total displacment*
				% simple summation of individual covariances
				% assuming independence
meas_t.R(1:3,4:9) = J*lambda;   % cross-covariance between displacement & velocity/attitude
meas_t.R(4:9,1:3) = lambda*J';
meas_t.R(4:6,4:6) = sigma2_t.uvw_V; % covariance of velocity measurement
meas_t.R(7:9,7:9) = sigma2_t.rph;   % covariance of attitude measurement
meas_t.varargin = {};
