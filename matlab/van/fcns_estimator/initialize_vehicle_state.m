function mindex_t = initialize_vehicle_state(nav_t,mindex_t,TheConfig)
%function mindex_t = initialize_vehicle_state(nav_t,mindex_t,TheConfig)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-12-2004      rme         Created & written.
%    04-21-2004      rme         Added phins bias estimates
%    09-01-2004      rme         Modified to use TheConfig.ProcessModel field
%    09-02-2004      rme         Added interpolation of nav XY to image time
%    10-18-2004      rme         Updated function name poll_navsensors.m to poll_nav_sensors.m
%                                Updated function call for process_measurements.m
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Modified to work with preallocated TheJournal
%    11-03-2004      rme         Added useEkf TheConfig check before calling computeOneNavCycle.m
%    12-27-2004      rme         Added path length calculation.
%    01-19-2005      rme         Cleaned up some baggage left over from path length calc code.
%    01-09-2006      rme         Added check for NaNs in nav_pose()
%    02-03-2006      rme         Updated nav_pose() to use nav_t.EST
%    05-01-2006      rme         Updated nav_pose() to check if 'PXF' field exists on nav_t
%                                Added TheJournal.t < t_stop condition

global TheJournal;

% shorthand for vehicle state index
Nv   = TheJournal.Index.Nv;
Np   = TheJournal.Index.Np;
Ne   = TheJournal.Index.Ne;
Xv_i = TheJournal.Index.Xv_i;
Xp_i = TheJournal.Index.Xp_i;
Xe_i = TheJournal.Index.Xe_i;
Xa_i = TheJournal.Index.Xa_i;

% initialize vehicle state to raw nav with large uncertainty
x_lv = nav_pose(nav_t,mindex_t,TheConfig);
TheJournal.Ekf.mu(Xv_i)          = 0;
TheJournal.Ekf.mu(Xv_i(Xp_i))    = x_lv;
TheJournal.Ekf.Sigma(Xp_i,Xp_i)  = blkdiag(1e-8*eye(2),1e6*eye(4));
TheJournal.Ekf.Sigma(Xe_i,Xe_i)  = 1e6*eye(Ne);
TheJournal.Eif.Lambda(Xv_i,Xv_i) = spdinverse(TheJournal.Ekf.Sigma(Xv_i,Xv_i));
TheJournal.Eif.eta(Xv_i)         = TheJournal.Eif.Lambda(Xv_i,Xv_i) * TheJournal.Ekf.mu(Xv_i);
TheJournal.Eif.mu(Xv_i)          = TheJournal.Ekf.mu(Xv_i);

% store original config EKF settings
useEkf = TheConfig.Estimator.useEkf;
inferenceEif = TheConfig.Estimator.inferenceEif;
TheConfig.Estimator.useEkf = on;
TheConfig.Estimator.inferenceEif = off;

% process nav up to the 1st camera meas to get a good initalization for
% the vehicle state elements
if isfield(nav_t,'PXF');
  t_stop = nav_t.PXF.rovtime(mindex_t.PXF);
else;
  t_stop = nav_t.RDI.rovtime(mindex_t.RDI)+10;
end;
while TheJournal.t < t_stop
    %===================================================
    % PROPOGATE STATE VECTOR FORWARD IN TIME
    %===================================================
    % initialize state based upon EKF state vector processing
    [sensorEvents,mindex_t] = computeOneNavCycle(nav_t,mindex_t,TheConfig,0);
end;

% restore config settings
TheConfig.Estimator.useEkf = useEkf;
TheConfig.Estimator.inferenceEif = inferenceEif;

% reset the x,y position values of the inital state to the
% x,y position reported by nav.  this makes comparison of x,y plots
% between nav and VAN convenient.
%---------------------------------------------------------------------------
x_lv = nav_pose(nav_t,mindex_t,TheConfig);
% set inital XY state to nav
TheJournal.Ekf.mu(Xp_i(1:2)) = x_lv(1:2);
% zero out the cross-covariance between vehicle XY and other states
TheJournal.Ekf.Sigma(Xp_i(1:2),Xv_i) = 0; 
TheJournal.Ekf.Sigma(Xv_i,Xp_i(1:2)) = 0;
% lock down inital XY state by setting covariance to be very small
TheJournal.Ekf.Sigma(Xp_i(1:2),Xp_i(1:2)) = 1e-8 * eye(2); % m^2

% unwrap intial state estimate so that it is easier to compare to other
% nav sensors
%---------------------------------------------------------------------
rph_est = TheJournal.Ekf.mu(Xp_i(4:6));
rph_est = funwrap([x_lv(4:6),rph_est],2);
TheJournal.Ekf.mu(Xp_i(4:6)) = rph_est(:,2);

% reset path length calculation
TheJournal.Pathlen.pathlen2    = 0;
TheJournal.Pathlen.pathlen3    = 0;
TheJournal.Pathlen.pathvec2(:) = 0;
TheJournal.Pathlen.pathvec3(:) = 0;
TheJournal.Pathlen.oldpos      = TheJournal.Ekf.mu(Xv_i(Xp_i));

% map EKF initalization to EIF, otherwise zero it out if not turned on in config file
if TheConfig.Estimator.useEif;
  TheJournal.Eif.Lambda(Xv_i,Xv_i) = sparse(spdinverse(TheJournal.Ekf.Sigma(Xv_i,Xv_i)));
  TheJournal.Eif.eta(Xv_i)         = TheJournal.Eif.Lambda(Xv_i,Xv_i)*TheJournal.Ekf.mu(Xv_i);
  TheJournal.Eif.mu(Xv_i)          = TheJournal.Ekf.mu(Xv_i);
else;
  TheJournal.Eif.Lambda   = [];
  TheJournal.Eif.eta      = [];
  TheJournal.Eif.mu       = [];
end;
% zero out EKF estimate if not turned on in config file
if ~TheConfig.Estimator.useEkf;
  TheJournal.Ekf.Sigma    = [];
  TheJournal.Ekf.mu       = [];
end;

%********************************************************************************
function x_lv = nav_pose(nav_t,mindex_t,TheConfig)


if isfield(nav_t,'LBL') && isfield(nav_t.LBL,'nx') && isfield(nav_t,'PXF');
  % interpolate to image time sample, also swap XY world for XY local-level
  ii = find(~isnan(nav_t.LBL.nx) & ~isnan(nav_t.LBL.ny));
  Xo = interp1(nav_t.LBL.rovtime(ii),nav_t.LBL.ny(ii),nav_t.PXF.rovtime(mindex_t.PXF));
  Yo = interp1(nav_t.LBL.rovtime(ii),nav_t.LBL.nx(ii),nav_t.PXF.rovtime(mindex_t.PXF));  
elseif isfield(nav_t,'EST') && isfield(nav_t,'PXF');
  % interpolate to image time sample, also swap XY world for XY local-level  
  ii = find(~isnan(nav_t.EST.x_ctl) & ~isnan(nav_t.EST.x_ctl));  
  Xo = interp1(nav_t.EST.rovtime(ii),nav_t.EST.y_ctl(ii),nav_t.PXF.rovtime(mindex_t.PXF));
  Yo = interp1(nav_t.EST.rovtime(ii),nav_t.EST.x_ctl(ii),nav_t.PXF.rovtime(mindex_t.PXF));
else;
  Xo = 0;
  Yo = 0;
end;

Zo = nav_t.PARO.depth(mindex_t.PARO);

if ~isempty(TheConfig.ObservationModel.PHINS)
  rph_nav = [nav_t.PHINS.roll_cooked(mindex_t.PHINS); ...
	     nav_t.PHINS.pitch_cooked(mindex_t.PHINS); ...
	     nav_t.PHINS.heading_cooked(mindex_t.PHINS)];
else
  rph_nav = [nav_t.RDI.roll_cooked(mindex_t.RDI); ...
	     nav_t.RDI.pitch_cooked(mindex_t.RDI); ...
	     nav_t.RDI.heading_cooked(mindex_t.RDI)];
end

x_lv = [Xo; Yo; Zo; rph_nav];
