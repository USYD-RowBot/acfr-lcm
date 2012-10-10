clear all;

% load jhu results
reload('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-2190.mat.gz');
TJi = TheJournal;
reload('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-2260.mat.gz');
TJj = TheJournal;
load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/results.mat')
load('/files1/processed/van/output/jhu04-6_gridsurvey3/nav_t.mat');
figure(5); clf;
plotTraj(TheJournal,nav_t,2000,3000,TheConfig);

% compute covariances
Xp_i = TJi.Index.Xp_i;
Xa_i = TJi.Index.Xa_i;
TJi.Eif.Sigma = spdinverse(TJi.Eif.Lambda(Xa_i,Xa_i));
Xa_i = TJj.Index.Xa_i;
TJj.Eif.Sigma = spdinverse(TJj.Eif.Lambda(Xa_i,Xa_i));
TJj.Eif.mu = TJj.Eif.Lambda(Xa_i,Xa_i) \ TJj.Eif.eta(Xa_i);

% select feature numbers
fni = find(TJj.Index.featureLUT==2160);
Xfi = TJj.Index.Xf_ii{fni};
fnj = find(TJj.Index.featureLUT==2260);
Xfj = TJj.Index.Xf_ii{fnj};

% true means
x_wi = TJj.Eif.mu(Xfi(Xp_i));
x_wj = TJj.Eif.mu(Xfj(Xp_i));

% true covariances
Pii  = TJj.Eif.Sigma(Xfi(Xp_i),Xfi(Xp_i));
Pij  = TJj.Eif.Sigma(Xfi(Xp_i),Xfj(Xp_i));
Pjj  = TJj.Eif.Sigma(Xfj(Xp_i),Xfj(Xp_i));
P    = [Pii, Pij; Pij' Pjj];

% convervative estimate for Pii since feature cov
% can only *decrease* with new measurements
PPii = TJi.Eif.Sigma(Xfi(Xp_i),Xfi(Xp_i));
PP   = [PPii, Pij; Pij' Pjj];

[R,p] = chol(PP);
if p ~= 0; error('PP not Pos Def!\n'); end;

% relative pose
[x_ji,J] = tail2tail(x_wj,x_wi);
Jj = J(:,1:6);
Ji = J(:,7:12);

% true relative pose covariance
P_x_ji = [Ji,Jj]*P*[Ji,Jj]';

% conservative relative pose covariance?
PP_x_ji = [Ji,Jj]*PP*[Ji,Jj]';


% generate a noisy 5 DOF camera meas
%------------------------------------------
t_dm = trans2dm(x_ji(1:3));
R_p_ji = 1e-6*eye(5);
p_ji = [t_dm(1:2); x_ji(4:6)] + chol(R_p_ji)'*randn(5,1);

% SUDO GLOBAL POSE MEASUREMENT
%====================================================
[x_wi_meas,J_x_wi_meas] = sudoPoseMeas0(x_wi,x_wj,p_ji);
Rc_x_wi_meas = J_x_wi_meas*blkdiag(PP,R_p_ji)*J_x_wi_meas';

% use covariance intersection to get a better estimate of global pose x_wi
%-----------------------------------------------------------
[P_x_wi_CI,omega] = covintersect(PPii,Rc_x_wi_meas,'trace');

alpha = 1 - 2*normcdf(-3);
k2 = chi2inv(alpha,2);
figure(2); clf;
mu = [0;0];
subplot(2,2,1); clear h;
h(1) = draw_ellipse(mu,Pjj(1:2,1:2),k2,0.7*[1,1,1]);
hold on;
h(2) = draw_ellipse(mu,Pii(1:2,1:2),k2,'k');
h(3) = draw_ellipse(mu,PPii(1:2,1:2),k2,'g');
legend(h,'P_{jj}','P_{ii}','UB P_{ii}',-1);
hold off;
axis equal tight;
title('Global Poses');

subplot(2,2,2); clear h;
h(1) = draw_ellipse(mu,P_x_ji(1:2,1:2),k2,'k');
hold on;
h(2) = draw_ellipse(mu,PP_x_ji(1:2,1:2),k2,'g');
hold off;
legend(h,'P_{x ji}','UB P_{x ji}',-1);
title('Relative Poses');
axis equal tight manual;

subplot(2,2,3); clear h;
h(1) = draw_ellipse(mu,Pii(1:2,1:2),k2,'k');
hold on;
h(2) = draw_ellipse(mu,PPii(1:2,1:2),k2,'g');
h(3) = draw_ellipse(mu,Rc_x_wi_meas(1:2,1:2),k2,'m');
h(4) = draw_ellipse(mu,P_x_wi_CI(1:2,1:2),k2,'c');
legend(h,'P_{x ii}','UB P_{x ii}','CamMeas P_{ii}','CI P_{x ii}',-1);
hold off;
title('Update Global Poses');
axis equal tight manual;
