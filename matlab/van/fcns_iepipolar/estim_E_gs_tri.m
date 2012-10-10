function [E,rph,t,Cov,p_est] = estim_E_gs_tri(t_o,rph_o,u1,v1,u2,v2,K)
%ESTIM_E_LM  Estimate essential matrix iteratively.
%  [E,rph,t,Cov] = ESTIM_E_GS_TRI(t_0,rph_0,u1,v1,u2,v2,K) estimates the
%  essential matrix E based upon initial guess of the [3x1] XYZ Euler
%  rotation angles rph_0 and the [3x1] baseline vector t_0.
%  Note that Covariance matrix is 5DOF using [rph;elev;azim] rep 
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12/30/2002      rme         Created and written.
%    01/02/2003      op          parametrized translation dir as elev,azim
%================================
% LEVENBERG-MARQUARDT MINIMIZATION
%================================


%decompose translation into elevation and azimuth angles
t = t_o/norm(t_o);
elev = asin(t(3));
azim = atan2(t(2),t(1));
np = length(u1);
invK = inv(K);

po = [rph_o; elev; azim];
options = optimset('Diagnostics','off','LargeScale','off','LevenbergMarquardt','on','Jacobian','off','Display','iter');

[p_est,resnorm,residuals,exitflag,output,lambda,J] = ...
    lsqnonlin(@reproj_cost,po,[],[],options,u1,v1,u2,v2,invK);

%calculate covariance of ML estimate
%Cov_p = pinv(J'*Cov_x*J)
% Cov_x assumed to be eye(np)
Cov = pinv(J'*J);

% decompose parameter vector
rph = p_est(1:3);
elev_est = p_est(4);
azim_est = p_est(5);
t(3) = sin(elev_est);
txy = cos(elev_est); % projection on xy plane of t
t(1) = txy*cos(azim_est);
t(2) = txy*sin(azim_est);



% compose essential matrix
E = skewsym(t)*rotxyz(rph);

% disp([t_o/norm(t_o), t/norm(t), [rph_o, rph]*180/pi]);

%================================================================================
function cost = reproj_cost(p,u1,v1,u2,v2,invK)

% decompose parameter vector
t  = zeros(3,1);
rph = p(1:3);
elev = p(4);
azim = p(5);
t(3) = sin(elev);
txy = cos(elev); % projection on xy plane of t
t(1) = txy*cos(azim);
t(2) = txy*sin(azim);

E = skewsym(t)*rotxyz(rph);

F = invK'*E*invK;

[e1,e2] = reprojection_error(F,u1,v1,u2,v2);

%cost = e1.^0.5 + e2.^0.5;
%cost = [e1.^0.5;e2.^0.5];
cost = [e1;e2];
%cost = ((u1e-u1).^2+(v1e-v1).^2).^0.5 + ((u2e-u2).^2+(v2e-v2).^2).^0.5;
