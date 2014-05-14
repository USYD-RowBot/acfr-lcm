function param_t = plot_ssa_xy(nav_t,TheConfig,fignum,param_t,TheJournal)
%PLOT_SSA_XY
%function param_t = plot_ssa_xy(nav_t,TheConfig,fignum,param_t,TheJournal)  
%
%   Options are controlled through a PARAM_T structure.  Two default
%   configurations exist from which you can modify.
%
%   PARAM_T = PLOT_SSA_XY(1); % vehicle config
%   PARAM_T = PLOT_SSA_XY(2); % camera  config
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-2003         rme         Created and written.
%    10-25-2003      rme         Restructed code organization.
%    02-09-2004      rme         Fixed bug in limits when plotting image numbers
%    04-05-2004      rme         Removed buffer from axis limits.
%    09-01-2004      rme         Updated to use TheJournal.Index.Xf_ii
%                                instead of Xfii

%================================
% default settings
%================================
if nargin == 1;
  if nav_t == 1
    param_t = veh_config;
  elseif nav_t == 2
    param_t = cam_config;
  end
  return;
elseif nargin == 0;
  param_t = veh_config;
  return;
end

if ~exist('TheJournal','var') || isempty(TheJournal)
  
else
  
end


%================================
% prepare figure
%================================
axe_buf = [0 70];
figure(fignum); %clf;
set(fignum,'DoubleBuffer','on','Nextplot','add');
%centerfig(fignum);
axis manual equal fill;
set(gca,'FontSize',param_t.fontsize);
grid on;
ylabel('North [m]','FontSize',param_t.fontsize);
xlabel('East [m]','FontSize',param_t.fontsize);
if param_t.show_cam_pose
  title('Estimated Camera Trajectory','FontSize',param_t.fontsize);
else
  title('Estimated Vehicle Trajectory','FontSize',param_t.fontsize);
end
set(gca,'NextPlot','add');

%================================
% initialize
%================================
% max axis limits
xmin_all = inf; xmax_all = -inf; ymin_all = inf; ymax_all = -inf;
% vehicle state vector is:
% Xv = [x_l, u, y_l, v, w_l, r, rdot, p, pdot, h, hdot]'
Xvi = [1:12]';
Xpi = [1:2:12]';
% local-level to world rotation matrix
Rwl = [0  1  0;
       1  0  0;
       0  0 -1];
% number of state snapshots
Ns = length(ssa_t.TheJournal);
% chi-squared variable corresponding to alpha confidence limit
k2 = chi2inv(param_t.alpha,2);

%================================
% plot dvl trajectory
%================================
if param_t.show_dvl_all && param_t.show_cam_pose
  % express dvl odometry from camera frame
  for cc=1:length(nav_t.RDI.rovtime)
    dvl = [nav_t.RDI.nx(cc); nav_t.RDI.ny(cc); 0];
    rph = [nav_t.RDI.roll_cooked(cc); nav_t.RDI.pitch_cooked(cc); nav_t.RDI.heading_cooked(cc)];
    cam = dvl + Rwl*rotxyz(rph)*TheConfig.SensorXform.PXF.tvs_v;
    nav_t.RDI.nx(cc) = cam(1);
    nav_t.RDI.ny(cc) = cam(2);
  end
elseif param_t.show_dvl_cur && param_t.show_cam_pose
  to = ssa_t.TheJournal{1}.t;
  tf = ssa_t.TheJournal{end}.t;
  ind = find(to <= nav_t.RDI.rovtime & nav_t.RDI.rovtime <= tf);  
  % express dvl odometry from camera frame
  for cc=ind'
    dvl = [nav_t.RDI.nx(cc); nav_t.RDI.ny(cc); 0];
    rph = [nav_t.RDI.roll_cooked(cc); nav_t.RDI.pitch_cooked(cc); nav_t.RDI.heading_cooked(cc)];
    cam = dvl + Rwl*rotxyz(rph)*TheConfig.SensorXform.PXF.tvs_v;
    nav_t.RDI.nx(cc) = cam(1);
    nav_t.RDI.ny(cc) = cam(2);
  end    
end
if param_t.show_dvl_all
  plot(nav_t.RDI.nx,nav_t.RDI.ny,'.', ...
       'Color',param_t.dvl_all_color,'MarkerSize',param_t.markersize);
end
unplot set;


%================================
% plot estimated trajectory
%================================
if param_t.make_movie
  aviobj = avifile(param_t.movie_name,'fps',param_t.fps);
  param_t.pause = 0;
end
if param_t.final_result
  ii_loop = Ns;
  param_t.show_all = true;
else
  ii_loop = 1:Ns;
end
for cc=1:2 % quick 1st pass to calculate max axis limits
  for ii=ii_loop % for each snapshot
    % remove previosly plotted ellipses
    unplot revert;
    % snapshot of state and covariance
    Xaug = ssa_t.TheJournal{ii}.Ekf.mu;
    Paug = ssa_t.TheJournal{ii}.Ekf.Sigma;
    Nf = ssa_t.TheJournal{ii}.Index.Nf;
    pos = []; % feature positions
    ex = []; ey = []; % confidence ellipse
    xy_pos = zeros(Nf,2); % pose mean
    lxt = []; lyt = []; % temporal links
    lxs = []; lys = []; % spatial links

    for jj=1:Nf % for each feature
      % feature indices in augmented state vector
      Xfi = ssa_t.TheJournal{ii}.Index.Xf_ii{jj};
      
      % extract feature pose and covariance
      Xf = Xaug(Xfi);
      rph = Xf(4:6);
      Pf = Paug(Xfi,Xfi);
    
      if param_t.show_cam_pose
	% map vehicle pose and uncertainty to camera frame
	Xf(1:3) = Xf(1:3) + rotxyz(rph)*TheConfig.SensorXform.PXF.tvs_v;
	a = rotz(rph(3))'*roty(rph(2))'*drotx(rph(1))*TheConfig.SensorXform.PXF.tvs_v;
	b = rotz(rph(3))'*droty(rph(2))'*rotx(rph(1))*TheConfig.SensorXform.PXF.tvs_v;
	c = drotz(rph(3))'*roty(rph(2))'*rotx(rph(1))*TheConfig.SensorXform.PXF.tvs_v;
	J = [eye(3), a, b, c;
	     zeros(3) eye(3)];
	Pf = J*Pf*J';
      end
      
      % rotate position from local-level to world frame (i.e. Y-North X-East)
      Xp_w = Rwl*Xf(1:3);
      Pp_w = Rwl*Pf(1:3,1:3)*Rwl';
      
      % store position
      xy_pos(jj,:) = Xp_w(1:2)';

      % calculate axis limits
      % all of traj history
      if xmin_all > Xp_w(1); xmin_all = Xp_w(1); end;
      if xmax_all < Xp_w(1); xmax_all = Xp_w(1); end;
      if ymin_all > Xp_w(2); ymin_all = Xp_w(2); end;
      if ymax_all < Xp_w(2); ymax_all = Xp_w(2); end;
      % follow vehicle with fixed window of view
      if jj==Nf
	win = 2.5;
	xmin_cur = Xp_w(1)-win; xmax_cur = Xp_w(1)+win;
	ymin_cur = Xp_w(2)-win; ymax_cur = Xp_w(2)+win;
      end
      if cc==1
	% only calculate max axis limits on first pass
	continue;
      end

      % calculate camera links
      if param_t.show_links && isfield(link_t,'vlinks')
	switch param_t.show_links
	 case 1
	  % show all links measured thus far
	  ind = find(link_t.vlinks(:,jj)==1);	  
	 case 2
	  % show only current links
	  if jj==Nf && jj <= size(link_t.vlinks,2)
	    ind = find(link_t.vlinks(:,jj)==1);
	  else
	    ind = [];
	  end
	end % switch
	for kk=1:length(ind)
	  if (jj-1) == ind(kk) % temporal
	    lxt = [lxt, [xy_pos(jj,1); xy_pos(ind(kk),1)]];
	    lyt = [lyt, [xy_pos(jj,2); xy_pos(ind(kk),2)]];
	  else % spatial
	    lxs = [lxs, [xy_pos(jj,1); xy_pos(ind(kk),1)]];
	    lys = [lys, [xy_pos(jj,2); xy_pos(ind(kk),2)]];
	  end
	end % for kk=1:length(ind)
      end % if show_links
      
      % calculate pose mean
      camnum = ssa_t.TheJournal{ii}.Index.featureLUT(jj);
      if camnum >= 0 % skip delayed doppler state
        % calculate delayed state ellipse 
	tmp = calc_ellipse(Xp_w(1:2),Pp_w(1:2,1:2),k2);
	% store ellipse x,y plot coordinates
	ex = [ex, tmp(:,1)];
	ey = [ey, tmp(:,2)];
	% store ellipse center coordinates and image number
	pos = [pos; [Xp_w(1), Xp_w(2), camnum]];
      end    
    
    end % for jj=1:Nf
    
    if cc==1 || isempty(ex)
      % only calculate max axis limits on first pass
      continue;
    end

    % plot current portion of dvl track
    if param_t.show_dvl_cur
      to = -inf; tf = to;
      if param_t.final_result
	to = ssa_t.TheJournal{1}.t;
	tf = ssa_t.TheJournal{end}.t;
      elseif ii > 1
	to = ssa_t.TheJournal{ii-1}.t;
	tf = ssa_t.TheJournal{ii}.t;
      end
      ind = find(to <= nav_t.RDI.rovtime & nav_t.RDI.rovtime <= tf);      
      plot(nav_t.RDI.nx(ind),nav_t.RDI.ny(ind),'.',...
	   'Color',param_t.dvl_cur_color,'MarkerSize',param_t.markersize);
      unplot set;
    end


    % plot camera links
    hdl_lt = line(lxt,lyt,'color',param_t.tl_color, ...
		  'LineWidth',param_t.linewidth);
    hdl_ls = line(lxs,lys,'color',param_t.sl_color, ...
		  'LineWidth',param_t.linewidth);
    % plot delayed state ellipses
    hdl_eds = plot(ex(:,1:end-1),ey(:,1:end-1), ...
		   'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
    hdl_ecs = plot(ex(:,end),ey(:,end), ...
		   'Color',param_t.cs_color,'MarkerSize',param_t.markersize);
    % plot ellipse centers
    hdl_pds = plot(pos(1:end-1,1),pos(1:end-1,2),'+', ...
		   'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
    hdl_pcs = plot(pos(end,1),pos(end,2),'+', ...
		   'Color',param_t.cs_color,'MarkerSize',param_t.markersize);
    
    % axis settings
    if param_t.show_all
      axis([xmin_all,xmax_all,ymin_all,ymax_all]);
      %axis([xmin_all-1,xmax_all+1,ymin_all-1,ymax_all+1]);
    else
      axis([xmin_cur,xmax_cur,ymin_cur,ymax_cur]);
      %axis([xmin_cur-1,xmax_cur+1,ymin_cur-1,ymax_cur+1]);      
    end

    % plot viewable image numbers
    if param_t.show_text
      limits = axis;
      ind = find(pos(:,1)>(limits(1)-1) & ...
		 pos(:,1)<(limits(2)+1) & ...
		 pos(:,2)>(limits(3)-1) & ...
		 pos(:,2)<(limits(4)+1));
      text_hdl = text(pos(ind,1)+0.1,pos(ind,2),strvcat(num2str(pos(ind,3))));
    end
    
    if ii==Ns && param_t.show_links == 2
      delete([hdl_lt;hdl_ls]);
    end
    
    drawnow;

    if param_t.make_movie
      F = getframe(gcf);
      if ~exist('nr','var')
	nr = size(F.cdata,1);
	nc = size(F.cdata,2);
      end
      % force all frames to be of equal size
      F = resize(F.cdata,nr,nc);
      aviobj = addframe(aviobj,F);
      if ii==Ns
	F = getframe(gcf);
	F = resize(F.cdata,nr,nc);
	for ll=1:(aviobj.Fps*5)
	  aviobj = addframe(aviobj,F);
	end
      end
    end
    
    if param_t.pause > 0
      pause(param_t.pause);
    elseif param_t.pause < 0
      pause;
    end
    
  end % for ii=1:Ns
end % for cc=1:L

if param_t.make_movie
  aviobj = close(aviobj);
  compressavi(aviobj);
end


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function param_t = veh_config
param_t.alpha = 0.99;
param_t.fps = 2.5;
param_t.make_movie = false;
param_t.final_result = false;
param_t.show_text = false;
param_t.axe_size = [520 480];
param_t.movie_name = 'xy.avi';
param_t.show_all = true;
param_t.show_cam_pose = false;
param_t.show_links = 2;
param_t.show_dvl_cur = true;
param_t.show_dvl_all = true;
param_t.pause = 0.5;
param_t.cs_color = 'm'; % current state
param_t.ds_color = 'c'; % delayed state
param_t.tl_color = 'g'; % temporal link
param_t.sl_color = 'r'; % spatial link
param_t.dvl_all_color  = [0.8 0.8 0.8];
param_t.dvl_cur_color  = 'y';
param_t.fontsize = 10;
param_t.markersize = 6;
param_t.linewidth = 0.5;

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function param_t = cam_config
param_t.alpha = 0.99;
param_t.fps = 2.5;
param_t.make_movie = false;
param_t.final_result = false;
param_t.show_text = false;
param_t.axe_size = [520 480];
param_t.movie_name = 'xy.avi';
param_t.show_all = true;
param_t.show_cam_pose = true;
param_t.show_links = 1;
param_t.show_dvl_cur = true;
param_t.show_dvl_all = true;
param_t.pause = 0.5;
param_t.cs_color = 'm'; % current state
param_t.ds_color = 'c'; % delayed state
param_t.tl_color = 'g'; % temporal link
param_t.sl_color = 'r'; % spatial link
param_t.dvl_all_color  = [0.8 0.8 0.8];
param_t.dvl_cur_color  = 'y';
param_t.fontsize = 10;
param_t.markersize = 6;
param_t.linewidth = 0.5;
