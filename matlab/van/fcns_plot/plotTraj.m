function Handles = plotTraj(dof,TheJournal,nav_t,iseq,plotpref,TheConfig);
%function Handles = plotTraj(dof,TheJournal,nav_t,iseq,plotpref,TheConfig)  
%
% plotpref = {'ekf','eif','eifbound'}
%
% HISTORY      WHO     WHAT
%----------    ----    -------------------------------------
% 11-04-2004   rme     Created and written.
% 11-29-2004   rme     Moved figure specification out of function.
% 12-03-2004   rme     Added TheConfig.Estimator.useEkf check.
% 12-06-2004   rme     Added sigmaEif and sigmaCI private functions.
% 01-19-2005   rme     Updated to not use sigmaCI
% 01-09-2006   rme     Added check for NaNs in nav_t.(sensorXY)

persistent initialized nx_interp ny_interp;
if isempty(initialized); initialized = false; end;

if true;
  % select best XY nav source
  if isfield(nav_t,'LBL');
    sensorXY = 'LBL';
  elseif isfield(nav_t,'RDI');
    sensorXY = 'RDI';
  else;
    error('no XY nav sensor');
  end;

  %if ~initialized;
    clf;
    ii = find(~isnan(nav_t.(sensorXY).nx) & ~isnan(nav_t.(sensorXY).ny));
    tmp = interp1(nav_t.(sensorXY).rovtime(ii), ...
		  [nav_t.(sensorXY).nx(ii),nav_t.(sensorXY).ny(ii)], ...
		  nav_t.PXF.rovtime);
    nx_interp = tmp(:,1);
    ny_interp = tmp(:,2);
  %else;
  %  unplot revert;
  %end;
  
  % find all XY DR measurements during image sequence
  ii = nav_t.PXF.camind.(sensorXY)(iseq(1)+1):nav_t.PXF.camind.(sensorXY)(iseq(end)+1);
  % plot nav
  hold on;
  plot(nav_t.(sensorXY).nx(ii), nav_t.(sensorXY).ny(ii), '.','color', 0.7*[1,1,1]);
  hold on;
  plot(nx_interp(iseq), ny_interp(iseq),'o','color', 0.7*[1,1,1]);
  plot(nx_interp(iseq), ny_interp(iseq),'x','color', 0.7*[1,1,1]);
  %unplot set;
end;

% plot link topology
param_t = plot_links(2);
param_t.show_links   = 1;
param_t.inflate      = 1;
param_t.colorscaled  = false;
param_t.show_text    = true;
param_t.fontsize     = 8;
plotpref = lower(plotpref);
if strcmp(plotpref,'ekf');
  Handles = plot_links(dof,param_t, TheJournal.Ekf.mu, TheJournal.Ekf.Sigma, ...
		       TheJournal.Links.vlinks, TheJournal.Index, TheConfig);
elseif strcmp(plotpref,'eif');
  Handles = plot_links(dof,param_t, TheJournal.Eif.mu, TheJournal.Eif.Sigma, ...
		       TheJournal.Links.vlinks, TheJournal.Index, TheConfig);
elseif strcmp(plotpref,'eifbound');
  Handles = plot_links(dof,param_t, TheJournal.Eif.mu, cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound), ...
		       TheJournal.Links.vlinks, TheJournal.Index, TheConfig);
elseif TheConfig.Estimator.useEkf;
  Handles = plot_links(dof,param_t, TheJournal.Ekf.mu, TheJournal.Ekf.Sigma, ...
		       TheJournal.Links.vlinks, TheJournal.Index, TheConfig);
else  TheConfig.Estimator.useEif;
  Handles = plot_links(dof,param_t, TheJournal.Eif.mu, cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound), ...
		       TheJournal.Links.vlinks, TheJournal.Index, TheConfig);  
end;

if param_t.colorscaled;
  delete(Handles.colorbar);
end;
axis normal equal;
grid on;
hold off;
initialized = true;

%axis(1e3*[-4.1615   -4.0892    3.7229    3.8416]);
%axis([ [-4.1455e3,-4.0792e3], [3.7804e3,3.8135e3]+25 ]);
%axis([ [-4163.9,-4125.8], [3724.8,3750.7] ]);

