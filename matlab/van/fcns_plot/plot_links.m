function varargout = plot_links(dof,param_t,mu,Sigma,vlinks,Index,TheConfig);
%PLOT_LINKS
%function varargout = plot_links(dof,param_t,mu,Sigma,vlinks,Index,TheConfig);
%
%   Options are controlled through a param_t structure.  Two default
%   configurations exist from which you can modify.
%
%       dof = {2,3}  
%   param_t = plot_links(1); % vehicle config
%   param_t = plot_links(2); % camera  config
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-04-2004      rme         Created and written from plot_ssa_xy.m
%    12-03-2004      rme         Added SigmaCol check.
%    01-05-2005      rme         Renamed index_t to Index.
%                                Added 2d/3d functionality.
%                                Added interpolated ellipse color functionality.
%    01-19-2005      rme         Updated to return colorbar handle.
%    01-20-2005      rme         Modified to plot links 1st, then ellipses
%    01-25-2005      rme         Removed unused options from param_t

% default settings
if nargin == 1;
  if dof == 1;
    varargout{1} = veh_config;
  elseif dof == 2;
    varargout{1} = cam_config;
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
chi2 = chi2inv(param_t.alpha,dof)*param_t.inflate;

% preallocate memory
%----------------------------------
% feature positions
[posX,posY,posZ] = deal(zeros(Nf,1));
% confidence ellipses
if dof == 2;
  [ellipseX,ellipseY,ellipseZ] = deal(zeros(1,Nf));
elseif dof == 3;
  [ellipseX,ellipseY,ellipseZ] = deal(zeros(1,Nf));
end;
% image numbers
imageNumber = zeros(Nf,1);
% temporal links
tmp = vlinks==1;
Nt  = nnz(diag(tmp,1));
[tlinkX,tlinkY,tlinkZ] = deal(zeros(2,Nf-1));
% spatial links
Ns  = nnz(tmp) - Nt;
[slinkX,slinkY,slinkZ] = deal(zeros(2,Ns));
% covariance metric
covmetric = zeros(Nf,1);
% handles structure
Handles.ellipse = [];
Handles.center  = [];
Handles.tlinks  = [];
Handles.slinks  = [];


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
  covmetric(fnj) = trace(Cov_x_wc(1:dof,1:dof))^0.5;
  
  % calculate the XY covariance ellipse
  if dof == 2;
    ii = [xi,yi];
    [x,y] = calculateEllipseXY(x_wc(ii),Cov_x_wc(ii,ii),chi2);
    % store the ellipse points
    n = length(x);
    ellipseX(1:n,fnj) = x;
    ellipseY(1:n,fnj) = y;
  elseif dof == 3;
    ii = [xi,yi,zi];
    [x,y,z] = calculateEllipseXYZ(x_wc(ii),Cov_x_wc(ii,ii),chi2);
    % store the ellipse points
    n = length(x);
    ellipseX(1:n,fnj) = x;
    ellipseY(1:n,fnj) = y;
    ellipseZ(1:n,fnj) = z;
  end;
  
  % store image number
  imageNumber(fnj) = Index.featureLUT(fnj);
  
  % store the XY mean
  posX(fnj) = x_wc(xi);
  posY(fnj) = x_wc(yi);
  posZ(fnj) = x_wc(zi);
    
  % store the temporal links
  if (fnj > 1) && (Index.featureLUT(fnj)~=inf) && (vlinks(fnj-1,fnj) == 1);
    tlinkX(:,fnj) = [posX(fnj); posX(fnj-1)];
    tlinkY(:,fnj) = [posY(fnj); posY(fnj-1)];
    tlinkZ(:,fnj) = [posZ(fnj); posZ(fnj-1)];
  else;
    tlinkX(:,fnj) = [nan; nan];
    tlinkY(:,fnj) = [nan; nan];
    tlinkZ(:,fnj) = [nan; nan];    
  end;
  
  % store the spatial links
  if (fnj > 1) && (Index.featureLUT(fnj)~=inf);
    fni = find(vlinks(1:fnj-2,fnj) == 1);
    Ni  = length(fni);
    if Ni > 0;
      ii = [1:Ni]+cc;
      slinkX(:,ii) = [repmat(posX(fnj),[1 Ni]); posX(fni)'];
      slinkY(:,ii) = [repmat(posY(fnj),[1 Ni]); posY(fni)'];
      slinkZ(:,ii) = [repmat(posZ(fnj),[1 Ni]); posZ(fni)'];
      cc = ii(end);
      if fnj==Nf;
	csi = ii; % current spatial index
      end;
    end;
  end;

end; %END: for fnj=1:length(Xf_ii)


%================================
% PREPARE FIGURE
%================================
fignum = gcf;
set(fignum,'DoubleBuffer','on');
NextPlot = get(gca,'NextPlot');
if ~ishold; clf; end;
set(gca,'NextPlot','Add');
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
grid on;

%=================================
% PLOT TOPOLOGY
%=================================
% plot camera links
switch param_t.show_links
case 1
 % show all links measured thus far
 if dof == 2;
   Handles.tlinks = line(tlinkX,tlinkY,'color',param_t.tl_color,'LineWidth',param_t.linewidth);
   Handles.slinks = line(slinkX,slinkY,'color',param_t.sl_color,'LineWidth',param_t.linewidth);
 elseif dof == 3;
   Handles.tlinks = line(tlinkX,tlinkY,tlinkZ,'color',param_t.tl_color,'LineWidth',param_t.linewidth);
   Handles.slinks = line(slinkX,slinkY,slinkZ,'color',param_t.sl_color,'LineWidth',param_t.linewidth);
 end;
case 2
 % show only current image links
 if dof == 2;
   Handles.tlinks = line(tlinkX(:,end),tlinkY(:,end),'color',param_t.tl_color,'LineWidth',param_t.linewidth);
   Handles.slinks = line(slinkX(:,csi),slinkY(:,csi),'color',param_t.sl_color,'LineWidth',param_t.linewidth);   
 elseif dof == 3;
   Handles.tlinks = line(tlinkX(:,end),tlinkY(:,end),tlinkZ(:,end),'color',param_t.tl_color,'LineWidth',param_t.linewidth);
   Handles.slinks = line(slinkX(:,csi),slinkY(:,csi),slinkZ(:,csi),'color',param_t.sl_color,'LineWidth',param_t.linewidth);
 end;
otherwise
 % don't plot any links
end % switch

% plot delayed state ellipses
if param_t.colorscaled;
  if dof == 2;
    Handles.ellipse = fill(ellipseX,ellipseY,covmetric');
  elseif dof == 3;
    n = sqrt(size(ellipseX,1));
    hold on;
    Handles.ellipse = zeros(Nf,1);
    for fni=1:Nf
      Handles.ellipse(fni) =  surf(reshape(ellipseX(:,fni),[n n]), ...
				   reshape(ellipseY(:,fni),[n n]), ...
				   reshape(ellipseZ(:,fni),[n n]), ...
				   repmat(covmetric(fni),[n n]));
    end;
    shading flat;
    hold off;
  end;
  % interpolate ellipse color based upon covariance metric
  cmap = cool;
  colormap(cmap);
  cmin = min(covmetric);
  cmax = max(covmetric);
  caxis([cmin,cmax+eps]); % guard against cmin==cmax
  if param_t.colorbar;
    Handles.colorbar = colorbar;
  end;
else;
  if dof == 2;
    % plot delayed state ellipses
    Handles.ellipse = plot(ellipseX(:,1:end-1),ellipseY(:,1:end-1), ...
			   'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
    hold on;
    Handles.ellipse(end+1) = plot(ellipseX(:,end),ellipseY(:,end), ...
				  'Color',param_t.cs_color,'MarkerSize',param_t.markersize);
    % plot ellipse centers
    Handles.center = plot(posX(1:end-1),posY(1:end-1),'+', ...
			  'Color',param_t.ds_color,'MarkerSize',param_t.markersize);
    Handles.center(end+1) = plot(posX(end),posY(end),'+', ...
				 'Color',param_t.cs_color,'MarkerSize',param_t.markersize);
  elseif dof == 3;
    n = sqrt(size(ellipseX,1));
    hold on;
    Handles.ellipse = zeros(Nf,1);
    for fni=1:Nf
      Handles.ellipse = surf(reshape(ellipseX(:,fni),[n n]), ...
			     reshape(ellipseY(:,fni),[n n]), ...
			     reshape(ellipseZ(:,fni),[n n]), ...
			     repmat(1,[n n]));
    end;
    shading flat;
    hold off;    
  end;
end;

set(gca,'FontSize',param_t.fontsize);
axis equal;
view(dof);
% plot viewable image numbers
if param_t.show_text
  limits = axis;
  if dof == 2;
    ii = find(posX > (limits(1)-1) & ...
	      posX < (limits(2)+1) & ...
	      posY > (limits(3)-1) & ...
	      posY < (limits(4)+1));
  elseif dof == 3;
    ii = find(posX > (limits(1)-1) & ...
	      posX < (limits(2)+1) & ...
	      posY > (limits(3)-1) & ...
	      posY < (limits(4)+1) & ...
	      posZ > (limits(5)+1) & ...
	      posZ < (limits(6)+1));
  end;
  ii = ii(1:end-1); % don't plot a number for current vehicle
  if dof == 2;
    Handles.text = text(posX(ii)+0.01, posY(ii), strvcat(num2str(imageNumber(ii))), ...
	 'fontsize',param_t.fontsize);
  elseif dof == 3;
    Handles.text = text(posX(ii)+0.01, posY(ii), posZ(ii), strvcat(num2str(imageNumber(ii))), ...
	 'fontsize',param_t.fontsize);    
  end;
end;
hold off;
set(fignum,'DoubleBuffer','off');
set(gca,'NextPlot',NextPlot);

if nargout;
  varargout{1} = Handles;
end;

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function param_t = veh_config
param_t.alpha = 1 - 2*normcdf(-3); % 3-sigma probability
param_t.inflate = 1;
param_t.colorscaled = 0;
param_t.show_text = false;
param_t.show_cam_pose = false;
param_t.show_links = 2;
param_t.cs_color = 'm'; % current state
param_t.ds_color = 'c'; % delayed state
param_t.tl_color = 'g'; % temporal link
param_t.sl_color = 'r'; % spatial link
param_t.fontsize = 3;
param_t.markersize = 4;
param_t.linewidth = 0.1;
param_t.colorbar = true;

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function param_t = cam_config
param_t.alpha = 1 - 2*normcdf(-3); % 3-sigma probability
param_t.inflate = 1;
param_t.colorscaled = 0;
param_t.show_text = false;
param_t.show_cam_pose = true;
param_t.show_links = 1;
param_t.cs_color = 'm'; % current state
param_t.ds_color = 'c'; % delayed state
param_t.tl_color = 'g'; % temporal link
param_t.sl_color = 'r'; % spatial link
param_t.fontsize = 3;
param_t.markersize = 4;
param_t.linewidth = 0.1;
param_t.colorbar = true;
