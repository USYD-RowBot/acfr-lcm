clear all;

% random global poses
Si = diag([0.1;0.1;0.1;1*DTOR;1*DTOR;360*DTOR]);
Sj = diag([.05;0.05;0.05;1*DTOR;1*DTOR;360*DTOR]);
x_wi = Si*randn(6,1);
x_wj = Sj*randn(6,1);

% SLAM filter covariances with x_wi being more uncertain that x_wj
S = blkdiag(Si,Sj);
Pt_joint = S*randcov(12)*S';   % true-joint covariance
Pt_x_wi = Pt_joint(1:6,1:6);
Pt_x_wj = Pt_joint(7:12,7:12);

% conservative covariance estimate for x_wi
Pc_x_wi = Pt_x_wi + Si*randcov(6)*Si';

% by replacing Pt_x_wi with Pc_x_wi_UB when can construct a
% convervative joint-covariance estimate Pc_joint
Pc_joint = Pt_joint;
Pc_joint(1:6,1:6) = Pc_x_wi;

% this test just illustrates that as long as Pc_x_wi - Pt_x_wi > 0
% is true, then Pc_joint will also be positive definite
[R,notPosDef] = chol(Pc_joint);
if notPosDef; 
  error('Pc_joint not Pos Def!\n'); 
end;

% RELATIVE POSES
%====================================================
% use compounding to get relative pose x_ij
[x_ij,J_x_ij] = tail2tail(x_wi,x_wj);
Pt_x_ij = J_x_ij*Pt_joint*J_x_ij'; % true covariance
Pc_x_ij = J_x_ij*Pc_joint*J_x_ij'; % conservative covariance

% use compounding to get relative pose x_ji
[x_ji,J_x_ji] = tail2tail(x_wj,x_wi);
J_x_ji = J_x_ji(:,[7:12,1:6]);
Pt_x_ji = J_x_ji*Pt_joint*J_x_ji'; % true covariance
Pc_x_ji = J_x_ji*Pc_joint*J_x_ji'; % conservative covariance


% CAMERA MEASURMENT
%====================================================
% generate a noisy 5 DOF camera measurement p_ji
t_dm = trans2dm(x_ji(1:3));
R_p_ji = 1e-6*eye(5);
p_ji = [t_dm(1:2); x_ji(4:6)] + chol(R_p_ji)'*randn(5,1);

% SUDO GLOBAL POSE MEASUREMENT
%====================================================
[x_wi_meas,J_x_wi_meas] = sudoPoseMeas0(x_wi,x_wj,p_ji);
Rc_x_wi_meas = J_x_wi_meas*blkdiag(Pc_joint,R_p_ji)*J_x_wi_meas';
%scale = norm(x_ji(1:3));
%R_scale = 1; % [m]
%[x_wi_meas,J_x_wi_meas] = sudoPoseMeas1(x_wj,p_ji,scale);
%Rc_x_wi_meas = J_x_wi_meas*blkdiag(Pt_x_wj,R_p_ji,R_scale)*J_x_wi_meas';

% use covariance intersection to get a better estimate of global pose x_wi
%-----------------------------------------------------------
[P_x_wi_CI,omega] = covintersect(Pc_x_wi,Rc_x_wi_meas,'trace');

figure(2); clf;
mu = [0;0];
subplot(2,2,1); clear h;
h(1) = draw_ellipse(mu,Pt_x_wj(1:2,1:2),1,0.7*[1,1,1]);
hold on;
h(2) = draw_ellipse(mu,Pt_x_wi(1:2,1:2),1,'k');
h(3) = draw_ellipse(mu,Pc_x_wi(1:2,1:2),1,'g');
legend(h,'P_t x_{wj}','P_t x_{wi}','P_c x_{wi}',-1);
hold off;
axis equal;
title('Global Poses');

subplot(2,2,2); clear h;
h(1) = draw_ellipse(mu,Pt_x_ji(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(mu,Pc_x_ji(1:2,1:2),1,'g');
hold off;
legend(h,'P_t x_{ji}','P_c x_{ji}',-1);
title('Relative Poses');
axis equal;

subplot(2,2,3); clear h;
h(1) = draw_ellipse(mu,Pt_x_wi(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(mu,Pc_x_wi(1:2,1:2),1,'g');
h(3) = draw_ellipse(mu,Rc_x_wi_meas(1:2,1:2),1,'m');
h(4) = draw_ellipse(mu,P_x_wi_CI(1:2,1:2),1,'c');
legend(h,'P_t x_{wi}','P_c x_{wi}','R_c x_{wi}','CI P_c x_{wi}',-1);
hold off;
title('Updated Global Pose x_{wi}');
axis equal;
