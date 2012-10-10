addpath(genpath('/files1/thesis/van'));
srcdir = '/files1/processed/van/output/titanic04/archive.20050120a.Titanic_Aplus/';
outdir = '/files1/processed/van/output/titanic04/archive.20050120a.Titanic_Aplus/framegrabs/';
Dirblist = dir([srcdir,'ssb-*.mat.gz']);
Diralist = dir([srcdir,'ssa-*.mat.gz']);

load([srcdir,'TheConfig']);

% plot topology prefs
param_t = plot_links(1);
param_t.show_links   = 1;
param_t.inflate      = 1;
param_t.colorscaled  = false;
param_t.show_text    = false;
param_t.fontsize     = 14;
dof = 2;

SigmaTrue = true;

for ii=768:length(Dirblist);
  fprintf('%d of %d\n',ii,length(Dirblist));
  fprintf('---------------------------------------------\n\n');
  
  % plot before
  reload([srcdir,Dirblist(ii).name],'TheJournal');
  figure(1);
  if SigmaTrue;
    TheJournal.Eif.Sigma = sigmaEiftmp(TheJournal,'block');
    Handles = plot_links(dof,param_t, TheJournal.Eif.mu, TheJournal.Eif.Sigma, ...
			 TheJournal.Links.vlinks, TheJournal.Index, TheConfig);  
  else;
    Handles = plot_links(dof,param_t, TheJournal.Eif.mu, cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound),...
			 TheJournal.Links.vlinks, TheJournal.Index, TheConfig);  
  end;
  drawnow;
  grab = getframe(1);
  save([outdir,'grab_',Dirblist(ii).name(1:8)],'grab');
  %mov(2*ii-1) = grab;
  
  % plot after
  reload([srcdir,Diralist(ii).name],'TheJournal');
  figure(1);
  if SigmaTrue;
    TheJournal.Eif.Sigma = sigmaEiftmp(TheJournal,'block');
    Handles = plot_links(dof,param_t, TheJournal.Eif.mu, TheJournal.Eif.Sigma, ...
			 TheJournal.Links.vlinks, TheJournal.Index, TheConfig);  
  else;
    Handles = plot_links(dof,param_t, TheJournal.Eif.mu, cell2Sigma(TheJournal,TheJournal.Eif.SigmaBound),...
			 TheJournal.Links.vlinks, TheJournal.Index, TheConfig);  
  end;
  drawnow;
  grab = getframe(1);
  save([outdir,'grab_',Diralist(ii).name(1:8)],'grab');
  %mov(2*ii) = grab;
end;

%save([outdir,'mov'],'mov');
