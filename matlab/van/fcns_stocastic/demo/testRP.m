clear all;
% generate random poses & covariance matrix
x_wi = randn(6,1);
x_wj = randn(6,1);

switch 'corr';
case 'rand';
 P   = randcov(12);
 Pii = P(1:6,1:6);
 Pij = P(1:6,7:12);
 Pjj = P(7:12,7:12);
case 'corr';
 F   = 1.001*eye(6);
 Pii = randcov(6);
 Pij = F*Pii;
 Pjj = F*Pii*F'+1e-6*eye(6);
 P   = [Pii, Pij; Pij' Pjj];
otherwise error('unknown selection');
end;
% generate a conservative/consistent Pjj cov estimate
isnotPosDef = true;
while isnotPosDef;
  PPjj = 2*randcov(6);
  [R,isnotPosDef] = chol(PPjj-Pjj);
end;
% we can solve a linear system for Pii,Pij exactly
% since feature covariance can only *decrease*
% our old covariance estiamte PPjj will serve as a
% consistent/conservative estimate
PP   = [Pii, Pij; Pij' PPjj];
[R1,p1] = chol(PP);
if p1 > 0;
  error('PP not Positive Definite!');
end

% relative pose
[x_ij,J] = tail2tail(x_wi,x_wj);
Ji = J(:,1:6);
Jj = J(:,7:12);

% true relative pose covariance
P_x_ij = J*P*J';

% conservative relative pose covariance?
PP_x_ij = J*PP*J';

% covariance intersection covariance estimate
%[P_x_ij_tr,omega_tr] = covintersect(Ji*Pii*Ji',Jj*Pjj*Jj','trace');
%[P_x_ij_det,omega_det] = covintersect(Ji*Pii*Ji',Jj*Pjj*Jj','det');


figure(1); clf;
mu = [0;0];
h(1) = draw_ellipse(mu,Pii(1:2,1:2),1,'k');
hold on;
h(2) = draw_ellipse(mu,Pjj(1:2,1:2),1,'k');
h(3) = draw_ellipse(mu,PPjj(1:2,1:2),1,'g');
h(4) = draw_ellipse(mu,P_x_ij(1:2,1:2),1,'b');
h(5) = draw_ellipse(mu,PP_x_ij(1:2,1:2),1,'r');
%h(5) = draw_ellipse(mu,P_x_ij_tr(1:2,1:2),1,'g');
%h(6) = draw_ellipse(mu,P_x_ij_det(1:2,1:2),1,'m');

hold off;
str = sprintf(['Fusion Test']);
title(str);
%legend(h,'P_{ii}','P_{jj}','True P_{x ij}','Est P_{x ij}','CI trace','CI det',-1)
legend(h,'P_{ii}','P_{jj}','PP_{jj}','True P_{x ij}','Est P_{x ij}',-1)
axis equal tight;


%disp([p1,omega_tr,omega_det]);
