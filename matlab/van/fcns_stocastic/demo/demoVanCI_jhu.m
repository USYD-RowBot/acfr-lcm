
inumi = 2080;
inumj = 2700;

% load jhu results
cmd = sprintf('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-%04d.mat.gz',inumi);
reload(cmd);
TJi = TheJournal;
cmd = sprintf('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-%04d.mat.gz',inumj);
reload(cmd);
TJj = TheJournal;
load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/results.mat')
load('/files1/processed/van/output/jhu04-6_gridsurvey3/nav_t.mat');
if ~exist('plotted','var') || plotted == 0;
  figure(5); clf;
  plotTraj(TheJournal,nav_t,2000,3000,TheConfig);
end;

% compute covariances
Xp_i = TJi.Index.Xp_i;
Xa_i = TJi.Index.Xa_i;
TJi.Eif.Sigma = spdinverse(TJi.Eif.Lambda(Xa_i,Xa_i));
Xa_i = TJj.Index.Xa_i;
TJj.Eif.Sigma = spdinverse(TJj.Eif.Lambda(Xa_i,Xa_i));
TJj.Eif.mu = TJj.Eif.Lambda(Xa_i,Xa_i) \ TJj.Eif.eta(Xa_i);

% select feature numbers
fni = find(TJj.Index.featureLUT==inumi);
Xfi = TJj.Index.Xf_ii{fni};
fnj = find(TJj.Index.featureLUT==inumj);
Xfj = TJj.Index.Xf_ii{fnj};

% "true" means
x_wi = TJj.Eif.mu(Xfi(Xp_i));
x_wj = TJj.Eif.mu(Xfj(Xp_i));

% "true" covariances
ii = [Xfi(Xp_i), Xfj(Xp_i)];
Pt_joint = TJj.Eif.Sigma(ii,ii);

% convervative estimate for Pii since feature cov
% can only *decrease* with new measurements
PPii = TJi.Eif.Sigma(Xfi(Xp_i),Xfi(Xp_i));

CIfusionDemo(x_wi,x_wj,Pt_joint,PPii);

plotted = 1;
