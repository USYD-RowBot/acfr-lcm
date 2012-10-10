function [uc,vc,residuals] = motion_center(y_o,u1,v1,u2,v2)

% compose vectors
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% compose motion direction vectors
mvec = x2 - x1;
mag = dot(mvec,mvec);
mvec = mvec./repmat(sqrt(mag),[2 1]);

% define optimization settings
options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
		   'MaxFunEvals',1e6, ...         % let it iterate indefinitely
		   'MaxIter',50, ...              % if initialized correctly, should coverge very quickly
		   'Display','off');             % level of display

% perform nonlinear minimization
[y_est,resnorm,residuals,exitflag,output,lambda,J] = ...
    lsqnonlin(@perpendicular_error,y_o,[],[],options,mvec,x1,x2);

uc = y_est(1);
vc = y_est(2);

%******************************************************************************
function cost_vector = perpendicular_error(y,mvec,x1,x2)
Npoints = length(x1);

% compute radial direction vector emanating from motion center out to
% midpoint
rvec = 0.5*(x1+x2) - repmat(y,[1 Npoints]);
mag  = sqrt(dot(rvec,rvec));
rvec = rvec./repmat(mag,[2 1]);

tvec = rvec([2,1],:);
tvec(1,:) = -tvec(1,:);

cost_vector = dot(mvec,rvec)';


% simple M-estimator limits max cost
MAX_ANGLE = 5*DTOR;
ii = find(abs(cost_vector) > MAX_ANGLE);
cost_vector(ii) = sign(cost_vector(ii)).*MAX_ANGLE;
