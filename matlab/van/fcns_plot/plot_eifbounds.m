function varargout = plot_eifbounds(TheJournal,TheConfig);
%function Handles = plot_eifbounds(TheJournal,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-14-2005      rme         Created and written.
%    01-19-2005      rme         Renamed to plot_eifbounds.m

param_t = plot_links(1);
param_t.show_dvl_cur = 0;
param_t.show_dvl_all = 0;
param_t.show_links   = 1;
param_t.inflate      = 30;
param_t.colorscaled  = true;
param_t.show_text    = 0;
param_t.fontsize     = 3;
param_t.colorbar     = false;

% initial bounds
Handles(1) = plot_links(2,param_t,TheJournal.Eif.mu,cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound0), ...
			TheJournal.Links.vlinks,TheJournal.Index,TheConfig);
hold on;

% kf updated bounds
param_t.show_links = 0; %turn off link plotting
Handles(2) = plot_links(2,param_t,TheJournal.Eif.mu,cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound), ...
			TheJournal.Links.vlinks,TheJournal.Index,TheConfig);

% eif recovered
Handles(3) = plot_links(2,param_t,TheJournal.Eif.mu,TheJournal.Eif.Sigma, ...
			TheJournal.Links.vlinks,TheJournal.Index,TheConfig);


set(Handles(1).ellipse,'EraseMode','background','FaceColor','r');
set(Handles(2).ellipse,'EraseMode','background','FaceColor',0.7*[1,1,1]);
set(Handles(3).ellipse,'EraseMode','background','FaceColor','g');


hold off;
axis normal equal;


if nargout;
  varargout{1} = Handles;
end;
