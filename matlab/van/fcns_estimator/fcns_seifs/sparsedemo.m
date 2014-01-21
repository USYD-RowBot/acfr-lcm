echo on;
if ~exist('loaddat','var') || loaddat ~= 1
  loaddat = 1;
  load /files1/processed/van/output/jhu04-6_gridsurvey3/CamResults/2000-3000wc.20040421.mat
  load /files1/processed/van/output/jhu04-6_gridsurvey3/nav_t;
end
  
% covariance matrix
Paug = ssa_t.TheJournal{end}.Paug;
Paug = Paug(1:end-6,1:end-6);

% information matrix
Haug = inv(Paug);

% normalized information matrix
thresh = 1e-10;
nHaug = rhomatrix(Haug);
ii = find(abs(nHaug) < thresh);

% modified information matrix
% set small normalized elements to zero
Haug_mod = Haug;
Haug_mod(ii) = 0;

% modified covariance matrix
Paug_mod = inv(Haug_mod);

% compare
laug = eig(Paug);
laug_mod = eig(Paug_mod);
disp('determinant');
disp([prod(laug) prod(laug_mod)]);
disp('Nth root determinant');
n = size(Paug,1);
disp([prod(laug.^(1/n)) prod(laug_mod.^(1/n))]);

disp('trace');
disp([trace(Paug) trace(Paug_mod)]);


figure(1); clf
param_t = plot_ssa_xy(1);
param_t.final_result = 1;
param_t.show_links = 1;
param_t.show_dvl_cur = 0;
param_t.show_dvl_all = 0;
plot_ssa_xy(nav_t,ssa_t,link_t,TheConfig,1,param_t);

figure(2); clf;
subplot(2,2,1);
plot_corrcoef(Paug,1);
title(sprintf('Paug'))

subplot(2,2,2);
plot_corrcoef(Haug,1);
title(sprintf('Haug'))
title(sprintf('Haug full=%.2f',nnz(Haug)/prod(size(Haug))));

subplot(2,2,3);
plot_corrcoef(Paug_mod,1);
title(sprintf('Paug\\_mod'))

subplot(2,2,4);
plot_corrcoef(Haug_mod,1)
title(sprintf('Haug\\_mod t=%g  full=%.2f',thresh,nnz(Haug_mod)/prod(size(Haug_mod))));
