addpath(genpath('/files1/thesis/van'));
srcdir = '/files1/processed/van/output/titanic04/archive.20050120a.Titanic_Aplus/';
outdir = '/files1/processed/van/output/titanic04/archive.20050120a.Titanic_Aplus/framegrabs/';
Dirblist = dir([srcdir,'ssb-*.mat.gz']);
Diralist = dir([srcdir,'ssa-*.mat.gz']);

aviobj = avifile('titanicEvolution.avi', ...
		 'fps',15);

for ii=1:30;%length(Dirblist);
  fprintf('\r%d of %d',ii,length(Dirblist));
  
  % load grab for ssb
  load([outdir,'grab_',Dirblist(ii).name(1:8)]);
  aviobj = addframe(aviobj,grab);
 
  % load grab for ssa
  load([outdir,'grab_',Dirblist(ii).name(1:8)]);
  aviobj = addframe(aviobj,grab);
end;
fprintf('\n');

keyboard;
aviobj = close(aviobj);
