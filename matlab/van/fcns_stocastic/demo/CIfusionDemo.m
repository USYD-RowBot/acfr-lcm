function CIfusion(x_wi,x_wj,Pt_joint,Pc_x_wi);

% "true" SLAM filter covariances
Pt_x_wi = Pt_joint(1:6,1:6);
Pt_x_wj = Pt_joint(7:12,7:12);

% by replacing Pt_x_wi with Pc_x_wi we can construct a
% convervative joint-covariance estimate Pc_joint
Pc_joint = Pt_joint;
Pc_joint(1:6,1:6) = Pc_x_wi;

% zero out Pij to show what the correlation buys us in using the relative
% pose for image registration
Pi_joint = Pc_joint;
Pi_joint(1:6,7:12) = 0;
Pi_joint(7:12,1:6) = 0;

% this test just illustrates that as long as Pc_x_wi - Pt_x_wi > 0
% is true, then Pc_joint will also be positive definite
[R,notPosDef] = chol(Pc_joint);
if notPosDef; 
  error('Pc_joint not Pos Def!');
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
Pi_x_ji = J_x_ji*Pi_joint*J_x_ji'; % conservative covariance & assume independence

% CAMERA MEASURMENT
%====================================================
[z_ji_predict,z_ji,R_z_ji,J_z_ji] = cameraMeas(x_wi,x_wj);

% SUDO GLOBAL POSE MEASUREMENTS
%====================================================
% sudo measurement of x_wi
[x_wi_meas,J_x_wi_meas] = sudoPoseMeas_xwi(x_wi,x_wj,z_ji);
Rc_x_wi_meas_nav = J_x_wi_meas*blkdiag(Pc_joint,R_z_ji)*J_x_wi_meas';
% sudo measurement of x_wj
[x_wj_meas,J_x_wj_meas] = sudoPoseMeas_xwj(x_wj,x_wi,z_ji);
Rc_x_wj_meas_nav = J_x_wj_meas*blkdiag(Pc_joint,R_z_ji)*J_x_wj_meas';

% use covariance intersection to get a better estimate of global pose x_wi
%-----------------------------------------------------------
[P_x_wi_CI_nav,omega_xwi] = covintersect(Pc_x_wi,Rc_x_wi_meas_nav,'trace');
[P_x_wj_CI_nav,omega_xwj] = covintersect(Pt_x_wj,Rc_x_wj_meas_nav,'trace');

% use an eif update to get a better estimate of global pose x_wi
Rinv = spdinverse(R_z_ji);
Ht_joint = spdinverse(Pt_joint);
% update
Hu_joint = Ht_joint + J_z_ji'*Rinv*J_z_ji;
Pu_joint = spdinverse(Hu_joint);

Pu_x_wi = Pu_joint(1:6,1:6);
Pu_x_wj = Pu_joint(7:12,7:12);

figure(2); plotresults(0);
figure(3); plotresults(1);

%================================================================================
function [z_ji_predict,z_ji_meas,R_ji,J] = cameraMeas(x_wi,x_wj)

[x_ji,minusJplus] = tail2tail(x_wj,x_wi);  

% generate a noisy 5 DOF camera measurement z_ji
[t_dm,Jb] = trans2dm(x_ji(1:3));

% measurment
R_ji = 1e-6*eye(5);
z_ji_predict = [t_dm(1:2); x_ji(4:6)];
z_ji_meas = z_ji_predict + chol(R_ji)'*randn(5,1);

% Jacobian of measurement w.r.t. relative camera pose
% i.e. d(z_ji)/d(x_cjci)
J_cjci = [Jb(1:2,:), zeros(2,3);
	  zeros(3),  eye(3)];

% fill in the augmented state Jacobian via chain-rule
J = J_cjci*minusJplus(:,[7:12,1:6]);


%==============================================================================
function plotresults(flag);

if flag==0
  [x_wi,x_wj,x_ji] = deal([0;0]);
else;
  x_wi = evalin('caller','x_wi');
  x_wj = evalin('caller','x_wj');
  x_ji = evalin('caller','x_ji');  
end;
omega_xwi = evalin('caller','omega_xwi');
omega_xwj = evalin('caller','omega_xwj');
Pt_x_wj = evalin('caller','Pt_x_wj');
Pt_x_wi = evalin('caller','Pt_x_wi');
Pc_x_wi = evalin('caller','Pc_x_wi');
Pu_x_wi = evalin('caller','Pu_x_wi');
Pu_x_wj = evalin('caller','Pu_x_wj');
Pt_x_ji = evalin('caller','Pt_x_ji');
Pc_x_ji = evalin('caller','Pc_x_ji');
Pi_x_ji = evalin('caller','Pi_x_ji');
Rc_x_wi_meas_nav = evalin('caller','Rc_x_wi_meas_nav');
Rc_x_wj_meas_nav = evalin('caller','Rc_x_wj_meas_nav');
P_x_wi_CI_nav = evalin('caller','P_x_wi_CI_nav');
P_x_wj_CI_nav = evalin('caller','P_x_wj_CI_nav');

subplot(2,2,1); clear h;
h(1) = draw_ellipse(x_wj(1:2),Pt_x_wj(1:2,1:2),1,'-','color',0.7*[1,1,1]);
hold on;
h(2) = draw_ellipse(x_wi(1:2),Pt_x_wi(1:2,1:2),1,'k');
h(3) = draw_ellipse(x_wi(1:2),Pc_x_wi(1:2,1:2),1,'g-.');
legend(h,'P_t x_{wj}','P_t x_{wi}','P_c x_{wi}',0);
hold off;
axis equal;
title('Global Poses');
axe1 = axis;

subplot(2,2,2); clear h;
h(1) = draw_ellipse(x_ji(1:2),Pt_x_ji(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(x_ji(1:2),Pc_x_ji(1:2,1:2),1,'g-.');
h(3) = draw_ellipse(x_ji(1:2),Pi_x_ji(1:2,1:2),1,'r-.');
hold off;
legend(h,'P_t x_{ji}','P_c x_{ji}','P_i x_{ji}',0);
title('Relative Poses');
axis equal;

subplot(2,2,3); clear h;
h(1) = draw_ellipse(x_wi(1:2),Pt_x_wi(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(x_wi(1:2),Pc_x_wi(1:2,1:2),1,'g-.');
h(3) = draw_ellipse(x_wi(1:2),Pu_x_wi(1:2,1:2),1,'b');
h(4) = draw_ellipse(x_wi(1:2),Rc_x_wi_meas_nav(1:2,1:2),1,'m-.');
h(5) = draw_ellipse(x_wi(1:2),P_x_wi_CI_nav(1:2,1:2),1,'c');
if flag;
  h(6) = draw_ellipse(x_wj(1:2),Pt_x_wj(1:2,1:2),1,'-','color',0.7*[1,1,1]);
  legend(h,'P_t','P_c','EKF','R_c nav','CI nav','EKF P_t','P_t x_{wj}',0);
else;
  legend(h,'P_t','P_c','EKF','R_c nav','CI nav',0);
end
axis(axe1);
hold off;
cmd = sprintf('Update Global Pose x_{wi} \\omega_{nav}=%.2f',omega_xwi);
title(cmd);

subplot(2,2,4); clear h;
h(1) = draw_ellipse(x_wj(1:2),Pt_x_wj(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(x_wj(1:2),Pu_x_wj(1:2,1:2),1,'b');
h(3) = draw_ellipse(x_wj(1:2),Rc_x_wj_meas_nav(1:2,1:2),1,'m-.');
h(4) = draw_ellipse(x_wj(1:2),P_x_wj_CI_nav(1:2,1:2),1,'c');
if flag;
  h(5) = draw_ellipse(x_wi(1:2),Pt_x_wi(1:2,1:2),1,'-','color',0.7*[1,1,1]);
  legend(h,'P_t','EKF','R_c nav','CI nav','P_t x_{wj}',0);
else;
  legend(h,'P_t','EKF','R_c nav','CI nav',0);
end
axis(axe1);
hold off;
cmd = sprintf('Update Global Pose x_{wj} \\omega_{nav}=%.2f',omega_xwj);
title(cmd);
