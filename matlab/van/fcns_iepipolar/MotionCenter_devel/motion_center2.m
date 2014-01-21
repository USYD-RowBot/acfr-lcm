function [y_est,residuals] = motion_center2(y_o,u1,v1,u2,v2)

% compose vectors
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% compose motion direction vectors
mvec = x2 - x1;
mag = dot(mvec,mvec);
mvec = mvec./repmat(sqrt(mag),[2 1]);

if length(y_o) < 3
  y_o = [y_o; 1];
end

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

y3 = y(3);
y = y(1:2)/y3;

% compute radial direction vector emanating from motion center out to
% midpoint
rvec = 0.5*(x1+x2) - repmat(y,[1 Npoints]);
mag  = sqrt(dot(rvec,rvec));
rvec = rvec./repmat(mag,[2 1]);

% compute the perpendicular of the radian vector which should be a
% tangent vector to the motion vector
tvec = rvec([2,1],:);
tvec(1,:) = -tvec(1,:);

%cost_vector = dot(mvec,rvec)';
cost_vector = acos(dot(mvec,rvec))'*RTOD - 90;

% simple M-estimator limits max cost
MAX_ANGLE = 5*10^abs(y3); % degrees
ii = find(abs(cost_vector) > MAX_ANGLE);
jj = find(abs(cost_vector) <= MAX_ANGLE);
cost_vector_orig = cost_vector;
cost_vector(ii) = sign(cost_vector(ii)).*MAX_ANGLE;
[cost_vector_orig';cost_vector']
[MAX_ANGLE,y3]

save /files1/thesis/van/mc2 ii jj cost_vector_orig cost_vector;
