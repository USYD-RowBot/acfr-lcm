clear all;

% load jhu results
reload('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-2130.mat.gz');
Tj2130 = TheJournal;
reload('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/ssa-2600.mat.gz');
Tj2600 = TheJournal;
load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/results.mat')
load('/files1/processed/van/output/jhu04-6_gridsurvey3/nav_t.mat');
figure(5); clf;
plotTraj(TheJournal,nav_t,2000,3000,TheConfig);

% compute covariances
Xa_i = Tj2130.Index.Xa_i;
Tj2130.Eif.Sigma = spdinverse(Tj2130.Eif.Lambda(Xa_i,Xa_i));
Xa_i = Tj2600.Index.Xa_i;
Tj2600.Eif.Sigma = spdinverse(Tj2600.Eif.Lambda(Xa_i,Xa_i));
Tj2600.Eif.mu = Tj2600.Eif.Lambda(Xa_i,Xa_i) \ Tj2600.Eif.eta(Xa_i);

% select feature numbers
fni = find(Tj2600.Index.featureLUT==2070);
Xfi = Tj2600.Index.Xf_ii{fni};
fnj = find(Tj2600.Index.featureLUT==2600);
Xfj = Tj2600.Index.Xf_ii{fnj};

% true means
x_wi = Tj2600.Eif.mu(Xfi);
x_wj = Tj2600.Eif.mu(Xfj);

% true covariances
Xp_i = Tj2600.Index.Xp_i;
Pii  = Tj2600.Eif.Sigma(Xfi(Xp_i),Xfi(Xp_i));
Pij  = Tj2600.Eif.Sigma(Xfi(Xp_i),Xfj(Xp_i));
Pjj  = Tj2600.Eif.Sigma(Xfj(Xp_i),Xfj(Xp_i));
P    = [Pii, Pij; Pij' Pjj];

% convervative estimate for Pii since feature cov
% can only *decrease* with new measurements
PPii = Tj2130.Eif.Sigma(Xfi(Xp_i),Xfi(Xp_i));
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
p_ji = [t_dm(1:2); x_ji(4:6)];
R_p_ji = 1e-6*eye(5);

% convert that to a 6 DOF relative pose meas using noisy altitude to set scale
%----------------------------------------------
[scale,Jd] = euclideanDistance(x_ji(1:3),[0;0;0]);
cov_scale = Jd(:,1:3)*PP_x_ji(1:3,1:3)*Jd(:,1:3)';
[x_ji_meas,Jmap] = five2six(p_ji,scale);
R_x_ji_meas = Jmap*blkdiag(R_p_ji,cov_scale)*Jmap';

% construct a conservative x_wi global position measurement
[x_wi_meas,JJ] = head2tail(x_wj,x_ji_meas);
P_x_wi_meas = JJ*blkdiag(Pjj,R_x_ji_meas)*JJ';

% use covariance intersection to get a better estimate of global pose x_wi
%-----------------------------------------------------------
[P_x_wi_CI,omega] = covintersect(PPii,P_x_wi_meas,'det');

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
h(3) = draw_ellipse(mu,R_x_ji_meas(1:2,1:2),k2,'m');
hold off;
legend(h,'P_{x ji}','UB P_{x ji}','UB Cam R_{x ji}',-1);
title('Relative Poses');
axis equal tight manual;

subplot(2,2,3); clear h;
h(1) = draw_ellipse(mu,Pii(1:2,1:2),k2,'k');
hold on;
h(2) = draw_ellipse(mu,PPii(1:2,1:2),k2,'g');
h(3) = draw_ellipse(mu,P_x_wi_meas(1:2,1:2),k2,'m');
h(4) = draw_ellipse(mu,P_x_wi_CI(1:2,1:2),k2,'c');
legend(h,'P_{x ii}','UB P_{x ii}','CamMeas P_{ii}','CI P_{x ii}',-1);
hold off;
title('Update Global Poses');
axis equal tight manual;
