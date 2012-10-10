function [y_est,residuals,FOEFLAG] = motioncenter_mest(y_o,u1,v1,u2,v2)

% put inital guess in homogenous form
if length(y_o) < 3
  y_o = [y_o; 1];
end

% compose vectors
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% compose motion unit direction vectors
m = x2 - x1;
m = m./repmat(sqrt(dot(m,m)),[2 1]);

% define optimization settings
options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
		   'MaxFunEvals',1e6, ...         % let it iterate indefinitely
		   'MaxIter',50, ...              % if initialized correctly, should coverge very quickly
		   'Display','off');             % level of display

% perform nonlinear minimization
[y_est] = lsqnonlin(@costfunction,y_o,[],[],options,m,x1,x2);

[residuals,FOEFLAG] = costfunction(y_est,m,x1,x2);

%******************************************************************************
function [residuals,FOEFLAG] = costfunction(y,m,x1,x2)
N = length(x1);

% dehomogenize
uc = y(1)/y(3);
vc = y(2)/y(3);

% compute radial direction vector emanating from 
% motion center out to midpoint
r = 0.5*(x1+x2) - repmat([uc;vc],[1 N]);
r = r./repmat(sqrt(dot(r,r)),[2 1]); % unit vector

% FOCUS OF EXPANSION
% compute the perpendicular of the radial vector which should be a
% orthogonal to the motion vector
P = [0 -1; 1 0];
r_perp = P*r;

residuals_com = acos(dot(m,r))'*RTOD - 90;
residuals_foe = acos(dot(m,r_perp))'*RTOD - 90;

% limit the residuals using a simple M-estimator
residuals_com = mestmc(residuals_com,y);
residuals_foe = mestmc(residuals_foe,y);

if residuals_com'*residuals_com < residuals_foe'*residuals_foe
  residuals = residuals_com;
  FOEFLAG = false;
else
  residuals = residuals_foe;
  FOEFLAG = true;
end
