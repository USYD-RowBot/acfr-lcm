function param_t = plot_links_xy(param_t,mu,Sigma,vlinks,Index,TheConfig);
%PLOT_LINKS_XY
%function param_t = plot_links_xy(param_t,mu,Sigma,vlinks,Index,TheConfig);
%
%   Options are controlled through a param_t structure.  Two default
%   configurations exist from which you can modify.
%
%   param_t = plot_links_xy(1); % vehicle config
%   param_t = plot_links_xy(2); % camera  config
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-04-2004      rme         Created and written from plot_ssa_xy.m
%    12-03-2004      rme         Added SigmaCol check.
%    01-05-2004      rme         Renamed index_t to Index.

% default settings
if nargin == 1;
  if param_t == 1;
    param_t = veh_config;
  elseif param_t == 2;
    param_t = cam_config;
  end;
  return;
end;

% shorthand index
Xp_i  = Index.Xp_i;    % pose elements index
Xf_ii = Index.Xf_ii;   % feature indices organized by feature number
Nf    = length(Xf_ii); % number of features
xi = Xp_i(1);
yi = Xp_i(2);
zi = Xp_i(3);
ri = Xp_i(4);
pi = Xp_i(5);
hi = Xp_i(6);

%=================================
% INITIALIZATION
%==================================
% static camera pose w.r.t. vehicle
x_vc = TheConfig.SensorXform.PXF.x_vs;
% local-level to world
x_wl = TheConfig.SensorXform.x_wl;
% chi-squared variable corresponding to alpha confidence limit
chi2 = chi2inv(param_t.alpha,2)*param_t.inflate;

% preallocate memory
%----------------------------------
% feature positions
posX = zeros(Nf,1);
posY = posX;
imageNumber = posX;
% confidence ellipses
ellipseX = zeros(20,Nf);
ellipseY = ellipseX;
% temporal links
tmp = vlinks==1;
Nt  = nnz(diag(tmp,1));
tlinkX = zeros(2,Nt);
tlinkY = tlinkX;
% spatial links
Ns  = nnz(tmp) - Nt;
slinkX = zeros(2,Ns);
slinkY = slinkX;
% covariance metric
covmetric = zeros(Nf,1);

%=================================
% COMPUTE TOPOLOGY GRAPH
%=================================
cc = 0;
for fnj=1:Nf; % for each feature
  % feature pose indices in augmented state vector
  Xf_i = Xf_ii{fnj}(Xp_i);
  
  % extract feature pose and covariance
  x_lv     = mu(Xf_i);
  Cov_x_lv = Sigma(Xf_i,Xf_i);
  
  % transform estimate to world frame
  [x_wv,Jplus] = head2tail(x_wl,x_lv);
  Cov_x_wv = Jplus(:,7:12)*Cov_x_lv*Jplus(:,7:12)';

  % transfer vehicle pose and uncertainty to camera frame
  if param_t.show_cam_pose;
    [x_wc,Jplus] = head2tail(x_wv,x_vc);
    Cov_x_wc = Jplus(:,1:6)*Cov_x_wv*Jplus(:,1:6)';
  else;
    x_wc = x_wv;
    Cov_x_wc = Cov_x_wv;
  end;
  
  % compute the covariance metric
  covmetric(fnj) = trace(Cov_x_wc(1:2,1:2))^0.5;
  
  % calculate the XY covariance ellipse
  ii = [xi,yi];
  [x,y] = calculateEllipseXY(x_wc(ii),Cov_x_wc(ii,ii),chi2);
  
  % store the ellipse points
  ellipseX(:,fnj) = x;
  ellipseY(:,fnj) = y;

  % store image number
  imageNumber(fnj) = Index.featureLUT(fnj);
  
  % store the XY mean
  posX(fnj) = x_wc(xi);
  posY(fnj) = x_wc(yi);
    
  % store the temporal links
  if (fnj > 1) && (vlinks(fnj-1,fnj) == 1);
    tlinkX(:,fnj) = [posX(fnj); posX(fnj-1)];
    tlinkY(:,fnj) = [posY(fnj); posY(fnj-1)];
  else;
    tlinkX(:,fnj) = [nan; nan];
    tlinkY(:,fnj) = [nan; nan];    
  end;
  
  % store the spatial links
  if (fnj > 1);
    fni = find(vlinks(1:fnj-2,fnj) == 1);
    Ni  = length(fni);
    if Ni > 0;
      ii = [1:Ni]+cc;
      slinkX(:,ii) = [repmat(posX(fnj),[1 Ni]); posX(fni)'];
      slinkY(:,ii) = [repmat(posY(fnj),[1 Ni]); posY(fni)'];
      cc = ii(end);
    end;
  end;

end; %END: for fnj=1:length(Xf_ii)


%================================
% PREPARE FIGURE
%================================
fignum = gcf;
set(fignum,'DoubleBuffer','on');
set(gca,'FontSize',param_t.fontsize);
ylabel('North [m]','FontSize',param_t.fontsize);
xlabel('East [m]','FontSize',param_t.fontsize);
if param_t.inflate ~= 1;
  string = sprintf('(inflated by a factor of %g)',param_t.inflate);
else;
  string = [];
end;
if param_t.show_cam_pose
  title(['Estimated Camera Trajectory 3-sigma ',string],'FontSize',param_t.fontsize);
else
  title(['Estimated Vehicle Trajectory 3-sigma ',string],'FontSize',param_t.fontsize);
end
axis equal;
grid on;

%=================================
% PLOT TOPOLOGY
%=================================
if param_t.colorscaled;
  % interpolate ellipse color based upon covariance metric
  cmin = min(covmetric);
  cmax = max(covmetric);
  cmap = cool;
  imap = round(interp1([cmin cmax],[1 length(cmap)],covmetric));

  % plot delayed state ellipses
  h = fill(ellipseX,ellipseY,covmetric');
  colormap(cmap);
  caxis([cmin,cmax]);
  colorbar;
else;
  % plot delayed state ellipses
  plot(ellipseX(:,1:end-1),ellipseY(:,1:end-1), ...
       'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
  hold on;
  plot(ellipseX(:,end),ellipseY(:,end), ...
       'Color',param_t.cs_color,'MarkerSize',param_t.markersize);

  % plot ellipse centers
  plot(posX(1:end-1),posY(1:end-1),'+', ...
       'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
  plot(posX(end),posY(end),'+', ...
       'Color',param_t.cs_color,'MarkerSize',param_t.markersize);
end;
  
% plot camera links
switch param_t.show_links
case 1
 % show all links measured thus far
 line(tlinkX,tlinkY,'color',param_t.tl_color,'LineWidth',param_t.linewidth);
 line(slinkX,slinkY,'color',param_t.sl_color,'LineWidth',param_t.linewidth); 
case 2
 % show only current temporal link
 line(tlinkX(:,end),tlinkY(:,end),'color',param_t.tl_color,'LineWidth',param_t.linewidth);
otherwise
 % don't plot any links
end % switch

% plot viewable image numbers
if param_t.show_text
  limits = axis;
  ii = find(posX > (limits(1)-1) & ...
	    posX < (limits(2)+1) & ...
	    posY > (limits(3)-1) & ...
	    posY < (limits(4)+1));
  ii = ii(1:end-1); % don't plot a number for current vehicle
  text(posX(ii)+0.01, posY(ii), strvcat(num2str(imageNumber(ii))), ...
       'fontsize',param_t.fontsize);
end
hold off;
set(fignum,'DoubleBuffer','off');

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function param_t = veh_config
param_t.alpha = 1 - 2*normcdf(-3); % 3-sigma probability
param_t.inflate = 1;
param_t.colorscaled = 0;
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
param_t.alpha = 1 - 2*normcdf(-3); % 3-sigma probability
param_t.inflate = 1;
param_t.colorscaled = 0;
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
