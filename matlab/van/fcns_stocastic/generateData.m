

datadir = '/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b';

tmp = dir([datadir,'/ssa-*.mat.gz']);
for fn = 1:length(tmp);
  fprintf('%d of %d\r',fn,length(tmp));
  reload(sprintf('%s/%s',datadir,tmp(fn).name));
  TJ(fn).fn = fn;
  Xa_i = TheJournal.Index.Xa_i;
  TJ(fn).Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
  TJ(fn).eta    = TheJournal.Eif.eta(Xa_i);
  TJ(fn).Sigma  = spdinverse(TJ(fn).Lambda);
  Tj(fn).mu     = TJ(fn).Lambda\TJ(fn).eta;
end;

Index = TheJournal.Index;

save([datadir,'/TJ.mat'],'TJ','Index');
