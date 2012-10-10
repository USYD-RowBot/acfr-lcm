function [y_est,residuals,FOEFLAG,osel] = motioncenter_mest(y_o,u1,v1,u2,v2)

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
[y_est] = lsqnonlin(@mccost,y_o,[],[],options,m,x1,x2,y_o);

[residuals,FOEFLAG,osel] = mccost(y_est,m,x1,x2,y_o);

