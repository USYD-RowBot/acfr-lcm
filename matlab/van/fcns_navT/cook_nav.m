function nav_t = cook_nav(nav_t,TheConfig)
%function nav_t = cook_nav(nav_t,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    20040310        rme         Created and written.
%    20040311        rme         Made more verbose and adding RDI hbias.
%    20040402        rme         Added RDI.beams & RDI.e flyer detection
%    20040412        rme         Modified to use funwrap.m instead of unwrap.m
%    20040517        rme         Fixed coordinate xform error with PHINS
%    20040902        rme         Incorporated TheConfig.Heuristic to control
%                                use of heuristic data filters
%    2004-11-14      rme         Added nav_t.RDI.beams filter
%    2004-11-14      rme         Modified rph_cooked calculation.
%    2004-11-15      rme         Added PHINS isLeftHanded flag to check is OCTANS is
%                                masquerading as PHINS
%    2004-11-22      rme         Made Rlr identity for OCTANS
%    2005-01-07      rme         Added RDI uvw zero velocity check.  All velocities equal
%                                to zero should be a rare condition, therefore use it to
%                                discard invalid DVL bottom pings which bt_status marks
%                                as good.
%    2006-01-05      rme         Removed rph2euler function call on raw PHINS rph data
%                                because the PHINS directly reports Euler angles, not
%                                inclinometer angles like RDI.
%    2006-01-09      rme         Added LBL section.
%                                Added PARO.depth_cooked calculation.

%======================================================
% REMOVE DUPLICATE LOGGER ENTRIES AS WELL AS DELAYED
% MEASUREMENTS TO SERIAL TRAFFIC PROBLEM
%======================================================
fields = fieldnames(nav_t);
for ii=1:length(fields);
  switch fields{ii};
   case {'STARTTIME','ENDTIME'};
    continue; % skip
   otherwise;
    tol = 0.01; % no sensor is running close to 100Hz
    % sensor data structure
    s_t = nav_t.(fields{ii});
    % duplicate entry index as well as delayed serial measurement index
    % are accounted for by tol
    tmp = find(abs(diff(s_t.rovtime)) < tol);
    s_t = rmindex(s_t,tmp);
    nav_t.(fields{ii}) = s_t;
  end; % switch fields{ii}
end; % for ii

  
%=================
% START RDI
%=================
if isfield(nav_t,'RDI');
  fprintf('======================================================\n');
  fprintf('%s: COOKING nav_t.RDI ...\n',mfilename);
  if isfield(nav_t.RDI,'u');
    % DVL RECORDED BODY FRAME VELOCITIES
    DVL_PRECOOKED = false;
    fprintf('DVL recorded body frame velocities.  Converting u,v,w to vx,vy,vz... ');
  else;
    % DVL RECORDED WORLD FRAME VELOCITIES
    DVL_PRECOOKED = true;
    fprintf('DVL recorded world frame velocities. Converting vx,vy,vz to u,v,w... ');
    nav_t.RDI.u = zeros(size(nav_t.RDI.rovtime));
    nav_t.RDI.v = zeros(size(nav_t.RDI.rovtime));
    nav_t.RDI.w = zeros(size(nav_t.RDI.rovtime));
  end;
    
  % put rdi meas in vehicle frame
  nav_t.RDI.roll_cooked    = zeros(size(nav_t.RDI.rovtime));
  nav_t.RDI.pitch_cooked   = zeros(size(nav_t.RDI.rovtime));
  nav_t.RDI.heading_cooked = zeros(size(nav_t.RDI.rovtime));
  nav_t.RDI.u_cooked       = zeros(size(nav_t.RDI.rovtime));
  nav_t.RDI.v_cooked       = zeros(size(nav_t.RDI.rovtime));
  nav_t.RDI.w_cooked       = zeros(size(nav_t.RDI.rovtime));  
  for ii=1:length(nav_t.RDI.rovtime);
    rph_raw = [nav_t.RDI.roll(ii); ...
	       nav_t.RDI.pitch(ii); ...
	       nav_t.RDI.heading(ii)] * DTOR;
    rph_raw = rph2euler(rph_raw); % correct RDI measured inclinometer pitch for Euler pitch 
    x_ls = [0;0;0;rph_raw];
    x_lv = head2tail(x_ls,inverse(TheConfig.SensorXform.RDI.x_vs));
    rph_cooked = x_lv(4:6);
    % store angles in radians
    nav_t.RDI.roll(ii)    = rph_raw(1);
    nav_t.RDI.pitch(ii)   = rph_raw(2);
    nav_t.RDI.heading(ii) = rph_raw(3);
    % store angles in vehicle frame    
    nav_t.RDI.roll_cooked(ii)    = rph_cooked(1);
    nav_t.RDI.pitch_cooked(ii)   = rph_cooked(2);
    nav_t.RDI.heading_cooked(ii) = rph_cooked(3);

    if DVL_PRECOOKED;
      % dvl cooked sensor frame velocities to world frame
      % put them back into sensor frame and vehicle frame (SEABED)
      Rlv = rotxyz(rph_cooked);
      Rvd = TheConfig.SensorXform.RDI.Rvs;
      Rwl = TheConfig.SensorXform.Rwl;
      Rwv = Rwl*Rlv;
      v_w = [nav_t.RDI.vx(ii); nav_t.RDI.vy(ii); nav_t.RDI.vz(ii)];
      v_v = Rwv'*v_w;
      v_d = Rvd'*v_v;
      % stuff sensor frame velocities
      nav_t.RDI.u(ii) = v_d(1);
      nav_t.RDI.v(ii) = v_d(2);
      nav_t.RDI.w(ii) = v_d(3);      
      % stuff vehicle frame velocities
      nav_t.RDI.u_cooked(ii) = v_v(1);
      nav_t.RDI.v_cooked(ii) = v_v(2);
      nav_t.RDI.w_cooked(ii) = v_v(3);
    else;
      % dvl measured sensor frame velocities
      % and did not cook them to world frame (JHU)
      uvw_d = [nav_t.RDI.u(ii); ...
	       nav_t.RDI.v(ii); ...
	       nav_t.RDI.w(ii)];
      Rvd = TheConfig.SensorXform.RDI.Rvs;
      uvw_v = Rvd * uvw_d;
      nav_t.RDI.u_cooked(ii) = uvw_v(1);
      nav_t.RDI.v_cooked(ii) = uvw_v(2);
      nav_t.RDI.w_cooked(ii) = uvw_v(3);      
    end; % if DVL_PRECOOKED
  end; % for ii
  nav_t.RDI.roll_cooked     = funwrap(nav_t.RDI.roll_cooked);
  nav_t.RDI.pitch_cooked    = funwrap(nav_t.RDI.pitch_cooked);
  nav_t.RDI.heading_cooked  = funwrap(nav_t.RDI.heading_cooked);
  fprintf('done\n');
  
  %=====================================================
  % HEURISTIC PRE-FILTER DATA FOR OUTLIER REJECTION
  %=====================================================
  % remove measurements which contain rdi velocity fliers
  if TheConfig.Heuristic.RDI_e_filt && isfield(nav_t.RDI,'e');
    % find indexs where error is high
    ind = find(abs(nav_t.RDI.e) > 0.005);
    fprintf('Heuristic e flyer removal.\n');
    % check if prev index is a 3 beam soln, if so remove it as well
    % (heuristic)
    ind_prev = ind-1;
    ind_prev = ind_prev(nav_t.RDI.beams(ind_prev) == 3);
    nav_t.RDI = rmindex(nav_t.RDI,[ind; ind_prev]);
  elseif TheConfig.Heuristic.RDI_uvw_flier_filt;
    fprintf('Heuristic u flyer removal.\n');
    ind = heuristic_flyer_detection(nav_t.RDI.u);
    nav_t.RDI = rmindex(nav_t.RDI,ind);
    fprintf('Heuristic v flyer removal.\n');
    ind = heuristic_flyer_detection(nav_t.RDI.v);
    nav_t.RDI = rmindex(nav_t.RDI,ind);
    fprintf('Heuristic w flyer removal.\n');
    ind = heuristic_flyer_detection(nav_t.RDI.w);
    nav_t.RDI = rmindex(nav_t.RDI,ind);
  end;
  
  % remove invalid bottom lock measurements
  if TheConfig.Heuristic.RDI_num_beams && isfield(nav_t.RDI,'beams');
    ind = find(nav_t.RDI.beams < 3);
    fprintf('Throwing out %d less than 3-beam solutions.\n',length(ind));
    nav_t.RDI = rmindex(nav_t.RDI,ind);    
  end;
  
  % check if uvw are all simultaneously equal to zero, if so throw out.
  if TheConfig.Heuristic.RDI_zero_velocity_filt;
    ind = find(nav_t.RDI.u==0 & nav_t.RDI.v==0 & nav_t.RDI.w==0);
    fprintf('Throwing out %d zero velocity solutions.\n',length(ind));
    nav_t.RDI = rmindex(nav_t.RDI,ind);
  end;

  
  % remove measurements which contain rdi heading fliers
  if TheConfig.Heuristic.RDI_rph_flier_filt;
    fprintf('Heuristic heading flyer removal.\n');  
    med = medfilt1(nav_t.RDI.heading_cooked,4);
    ind = find(abs(nav_t.RDI.heading_cooked-med) > 1.5*DTOR);
    nav_t.RDI = rmindex(nav_t.RDI,ind);
    % fliers may have caused angle unwrapping to be incorrect, unwrap
    % heading angle again to make sure it is continuous
    nav_t.RDI.roll_cooked    = funwrap(nav_t.RDI.roll_cooked);
    nav_t.RDI.pitch_cooked   = funwrap(nav_t.RDI.pitch_cooked);
    nav_t.RDI.heading_cooked = funwrap(nav_t.RDI.heading_cooked);  
  end;
  
end; % if isfield(nav_t,'RDI')
%=================
% END RDI
%=================

  
%=================
% START XBOW
%=================
if isfield(nav_t,'XBOW');
  fprintf('======================================================\n');    
  fprintf('%s: COOKING nav_t.XBOW ...\n',mfilename);  
  if TheConfig.Heuristic.XBOW_fix_parse;
    % correct parsing error in XBOW roll/pitch rate measurements
    fprintf('Heuristic parsing error correction.\n');    
    tmp1 = find(nav_t.XBOW.rr > 150);
    nav_t.XBOW.rr(tmp1) = nav_t.XBOW.rr(tmp1) - 300;
    tmp1 = find(nav_t.XBOW.pr > 150);
    nav_t.XBOW.pr(tmp1) = nav_t.XBOW.pr(tmp1) - 300;
  end;
  % put xbow meas in vehicle frame 
  nav_t.XBOW.roll_cooked    = zeros(size(nav_t.XBOW.rovtime));
  nav_t.XBOW.pitch_cooked   = zeros(size(nav_t.XBOW.rovtime));
  nav_t.XBOW.heading_cooked = zeros(size(nav_t.XBOW.rovtime));
  nav_t.XBOW.rr_cooked      = zeros(size(nav_t.XBOW.rovtime));
  nav_t.XBOW.pr_cooked      = zeros(size(nav_t.XBOW.rovtime));
  nav_t.XBOW.hr_cooked      = zeros(size(nav_t.XBOW.rovtime));
  for ii=1:length(nav_t.XBOW.rovtime);
    rph_raw = [nav_t.XBOW.roll(ii); ...
	       nav_t.XBOW.pitch(ii); ...
	       nav_t.XBOW.heading(ii)] * DTOR;
    x_ls = [0;0;0;rph_raw];
    x_lv = head2tail(x_ls,inverse(TheConfig.SensorXform.RDI.x_vs));
    rph_cooked = x_lv(4:6);
    % store angles in radians
    nav_t.XBOW.roll(ii)    = rph_raw(1);
    nav_t.XBOW.pitch(ii)   = rph_raw(2);
    nav_t.XBOW.heading(ii) = rph_raw(3);
    % store angles in vehicle frame    
    nav_t.XBOW.roll_cooked(ii)    = rph_cooked(1);
    nav_t.XBOW.pitch_cooked(ii)   = rph_cooked(2);
    nav_t.XBOW.heading_cooked(ii) = rph_cooked(3);
        
    rphdot_raw = [nav_t.XBOW.rr(ii); ...
		  nav_t.XBOW.pr(ii); ...
		  nav_t.XBOW.hr(ii)] * DTOR;
    rphdot_cooked = TheConfig.SensorXform.XBOW.Rvs*rphdot_raw;
    % store rates in radians
    nav_t.XBOW.rr(ii) = rphdot_raw(1);
    nav_t.XBOW.pr(ii) = rphdot_raw(2);
    nav_t.XBOW.hr(ii) = rphdot_raw(3);
    % store rates in vehicle frame
    nav_t.XBOW.rr_cooked(ii) = rphdot_cooked(1);
    nav_t.XBOW.pr_cooked(ii) = rphdot_cooked(2);
    nav_t.XBOW.hr_cooked(ii) = rphdot_cooked(3);        
  end;
  nav_t.XBOW.roll_cooked     = funwrap(nav_t.XBOW.roll_cooked);
  nav_t.XBOW.pitch_cooked    = funwrap(nav_t.XBOW.pitch_cooked);
  nav_t.XBOW.heading_cooked  = funwrap(nav_t.XBOW.heading_cooked);
end; % if isfield(nav_t,'XBOW')
%=================
% END XBOW
%=================

%=================
% OCTANS HACK!!!
%=================
if isfield(nav_t,'OCTANS');
  nav_t.PHINS = nav_t.OCTANS;
end;

%=================
% START PHINS
%=================
if isfield(nav_t,'PHINS');
  fprintf('======================================================\n');    
  fprintf('%s: COOKING nav_t.PHINS ...\n',mfilename);
  nav_t.PHINS.roll_cooked    = zeros(size(nav_t.PHINS.rovtime));
  nav_t.PHINS.pitch_cooked   = zeros(size(nav_t.PHINS.rovtime));
  nav_t.PHINS.heading_cooked = zeros(size(nav_t.PHINS.rovtime));
  % put phins meas in vehicle frame
  isLeftHanded = strncmp(func2str(TheConfig.ObservationModel.PHINS),'om_phins',8)
  for ii=1:length(nav_t.PHINS.rovtime);
    if isLeftHanded;
      % raw sensor meas, corrected for left-handed coordinate frame heading meas, i.e. Phins
      rph_raw = [ nav_t.PHINS.roll(ii); ...
		  nav_t.PHINS.pitch(ii); ...
		 -nav_t.PHINS.heading(ii)] * DTOR;
      % rotation matrix from phins reference frame to local-level
      Rlr = [1 0 0; 0 -1 0; 0 0 -1];
    else;
      % raw sensor meas, i.e. Octans
      rph_raw = [ nav_t.PHINS.roll(ii); ...
		  nav_t.PHINS.pitch(ii); ...
		  nav_t.PHINS.heading(ii)] * DTOR;
      % rotation matrix from octans reference frame to local-level is identity
      Rlr = eye(3);
    end;
    % vehicle attitude in local-level frame
    Rlv = Rlr * rotxyz(rph_raw) * TheConfig.SensorXform.PHINS.Rvs';
    rph_cooked = rot2rph(Rlv);
    % store angles in radians
    nav_t.PHINS.roll(ii)    = rph_raw(1);
    nav_t.PHINS.pitch(ii)   = rph_raw(2);
    nav_t.PHINS.heading(ii) = rph_raw(3);
    % store angles in vehicle frame
    nav_t.PHINS.roll_cooked(ii)    = rph_cooked(1);
    nav_t.PHINS.pitch_cooked(ii)   = rph_cooked(2);
    nav_t.PHINS.heading_cooked(ii) = rph_cooked(3);
  end;
  nav_t.PHINS.roll_cooked    = funwrap(nav_t.PHINS.roll_cooked);
  nav_t.PHINS.pitch_cooked   = funwrap(nav_t.PHINS.pitch_cooked);
  nav_t.PHINS.heading_cooked = funwrap(nav_t.PHINS.heading_cooked);
end; % if isfield(nav_t,'PHINS')
%=================
% END PHINS
%=================
  
%=================
% START KVH
%=================
if isfield(nav_t,'KVH');
  fprintf('======================================================\n');    
  fprintf('%s: COOKING nav_t.KVH ...\n',mfilename);  
  nav_t.KVH.roll    = zeros(size(nav_t.KVH.rovtime));
  nav_t.KVH.pitch   = zeros(size(nav_t.KVH.rovtime));
  nav_t.KVH.heading = zeros(size(nav_t.KVH.rovtime));  
  % kvh meas is already cooked, put back into sensor frame
  for ii=1:length(nav_t.KVH.rovtime);
    rph_cooked = [nav_t.KVH.roll(ii); ...
		  nav_t.KVH.pitch(ii); ...
		  nav_t.KVH.heading(ii)] * DTOR;
    x_ls = [0;0;0;rph_raw];
    x_lv = head2tail(x_ls,inverse(TheConfig.SensorXform.RDI.x_vs));
    rph_cooked = x_lv(4:6);
    % store angles in radians
    nav_t.KVH.roll(ii)    = rph_raw(1);
    nav_t.KVH.pitch(ii)   = rph_raw(2);
    nav_t.KVH.heading(ii) = rph_raw(3);
    % store angles in vehicle frame
    nav_t.KVH.roll_cooked(ii)    = rph_cooked(1);
    nav_t.KVH.pitch_cooked(ii)   = rph_cooked(2);
    nav_t.KVH.heading_cooked(ii) = rph_cooked(3);
  end;
  nav_t.KVH.roll_cooked     = funwrap(nav_t.KVH.roll_cooked);
  nav_t.KVH.pitch_cooked    = funwrap(nav_t.KVH.pitch_cooked);
  nav_t.KVH.heading_cooked  = funwrap(nav_t.KVH.heading_cooked);
end; % if isfield(nav_t,'KVH')
%=================
% END KVH
%=================

%=================
% START PARO
%=================
if isfield(nav_t,'PARO');
  fprintf('======================================================\n');    
  fprintf('%s: COOKING nav_t.PARO ...\n',mfilename);
  if TheConfig.Heuristic.PARO_serial_filt;
    %=====================================================
    % HEURISTIC PRE-FILTER DATA FOR OUTLIER REJECTION
    %=====================================================
    fprintf('Heuristic depth flyer removal\n');  
    % paro, after gap in data, old measurement prior to gap was buffered,
    % remove these delayed measurements
    tmp1 = find(diff(nav_t.PARO.rovtime)>3.0)+1; % outlier index
    % check to see if outliers came in pairs
    tmp2 = find(abs(nav_t.PARO.depth(tmp1)-nav_t.PARO.depth(tmp1+1))<0.025);
    tmp1 = [tmp1;tmp1(tmp2)+1];
    nav_t.PARO = rmindex(nav_t.PARO,tmp1);
  end;
  % interpolate vehicle attitude to PARO time samples
  if isfield(nav_t,'PHINS');
    rphi = interp1(nav_t.PHINS.rovtime, ...
		   [nav_t.PHINS.roll_cooked, nav_t.PHINS.pitch_cooked, nav_t.PHINS.heading_cooked], ...
		   nav_t.PARO.rovtime);
  else;
    rphi = interp1(nav_t.RDI.rovtime, ...
		   [nav_t.RDI.roll_cooked, nav_t.RDI.pitch_cooked, nav_t.RDI.heading_cooked], ...
		   nav_t.PARO.rovtime);    
  end;
  nav_t.PARO.depth_cooked = zeros(size(nav_t.PARO.depth));
  for ii=1:length(nav_t.PARO.rovtime);
    % compute depth at vehicle frame
    x_vs = TheConfig.SensorXform.PARO.x_vs;            % paro frame w.r.t. vehicle-frame
    x_ls = [0, 0, nav_t.PARO.depth(ii), rphi(ii,:)]';  % paro frame w.r.t. local-level
    x_lv = head2tail(x_ls, inverse(x_vs));             % vehicle frame w.r.t. local-level
    nav_t.PARO.depth_cooked(ii)  = x_lv(3);            % depth of vehicle in local-level
  end;
end; % if isfield(nav_t,'PARO')
%=================
% END PARO
%=================

%=================
% START LBL
%=================
if isfield(nav_t,'LBL.owtt');
  fprintf('======================================================\n');    
  fprintf('%s: COOKING nav_t.LBL ...\n',mfilename);
  Param = lblFixAutoParam(nav_t,TheConfig);
  [xyzFix,bused] = lblFixAuto(Param);
  if ~all(isnan(xyzFix(:,1)));
    nav_t.LBL.nx = xyzFix(:,1);
    nav_t.LBL.ny = xyzFix(:,2);
    nav_t.LBL.nz = xyzFix(:,3);
    nav_t.LBL.bused = bused;
  end;
end; % if isfield(nav_t,'LBL')
%=================
% END LBL
%=================

%*********************************************************************
function ind = heuristic_flyer_detection(X);
med = medfilt1(X,3);
dif = X - med;
med = medfilt1(dif,3);
ind = find(abs(dif) > 5*abs(med));
