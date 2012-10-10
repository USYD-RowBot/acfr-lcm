function measout_t = relocalize(meas_t,TheConfig);

global TheJournal;  

n = length(meas_t);
if n < 2;
  fprintf('***%s: warning, need 2 or more camera measurements to triangulate and relocalize!\n',mfilename);
  return;
end;

x_vc    = TheConfig.SensorXform.PXF.x_vs;
fnj     = meas_t(1).fnj;
Xp_i    = TheJournal.Index.Xp_i;
Xfj_i   = TheJournal.Index.Xf_ii{fnj};
x_lvj_o = TheJournal.Eif.mu(Xfj_i(Xp_i)); 

% define optimization settings
Options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LargeScale','on', ...         % use large-scale algorithm
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
		   'Jacobian','off', ...          % jacobian function defined by user
		   'MaxFunEvals',1e6, ...         % let it iterate indefinitely
		   'MaxIter',200, ...             % if initialized correctly, should coverge very quickly
		   'Display','iter');             % level of display


% perform nonlinear minimization
[x_lvj,resnorm,residuals,exitflag,output,lambda,J] = ...
    lsqnonlin(@localization_error,x_lvj_o,[],[],Options,meas_t,Xp_i,x_vc);

fprintf('==>%s: Relocalized x_lvj\n',mfilename);
disp([x_lvj_o, x_lvj]);

% stuff the measurement and covariance
measout_t.omfhandle = @om_lbl_xyz;
measout_t.omfhandleString = func2str(measout_t.omfhandle);
measout_t.isaDelayedStateMeas  = false;
measout_t.z        = x_lvj(1:3);
measout_t.R        = 3*eye(3);
measout_t.varargin = {zeros(6,1)}; % sensor pose w.r.t. to vehicle frame

for ii=1:n;
  fni = meas_t(ii).fni;
  if fnj-fnj == 1;
    measout_t(2). mes_t(ii); % also stuff temporal meas as a normal camera meas
    break;
  end;
end;

%================================================================================
function errorVector = localization_error(x_lvj,meas_t,Xp_i,x_vc);

global TheJournal;

n = length(meas_t);
errorVector = [];
for ii=1:n;
  % pose of vehicle i in local-level
  fni   = meas_t(ii).fni;
  fnj   = meas_t(ii).fnj;
  x_lvi = TheJournal.Index.Xf_ii{fni}(Xp_i);
  
  % skip temporal pair since fni needs to be relocalized as well
  if fnj-fni == 1;
    continue;
  end;
  
  % predicted relative camera pose
  x_cjci = relative_sensor_pose(x_lvj,x_lvi,x_vc);
  
  % decompose into relative pose parameters
  t   = x_cjci(1:3);
  rph = x_cjci(4:6);

  %======================================================
  % camera 5 DOF observation model
  %======================================================
  % baseline direction and associated Jacobian
  [b,Jb] = trans2dm(t);

  % predicted measurement z_ji
  z_predict = [b(1:2); rph]; % [az,el,r,p,h]
  
  errorVector = [errorVector; meas_t(ii).z-z_predict];
end;
