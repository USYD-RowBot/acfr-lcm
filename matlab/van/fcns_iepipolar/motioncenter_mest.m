function [y_est,residuals,FOEFLAG,osel] = motioncenter_mest(y_o,u1,v1,u2,v2)
%function [y_est,residuals,FOEFLAG,osel] = motioncenter_mest(y_o,u1,v1,u2,v2)

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


%==========================================================================================
function [residuals,FOEFLAG,osel] = mccost(y,m,x1,x2,y_o)
N = length(x1);

% dehomogenize
uc = y(1)/y(3);
vc = y(2)/y(3);

% compute radial direction vector emanating from 
% motion center out to midpoint
r = 0.5*(x1+x2) - repmat([uc;vc],[1 N]);
r = r./repmat(sqrt(dot(r,r)),[2 1]); % unit vector

% compute the perpendicular of the radial vector which should be a
% orthogonal to the motion vector
P = [0 -1; 1 0];
r_perp = P*r;

% Center of Motion
% motion vector should be perpendicular to radial vector
mdotr = dot(m,r)';
residuals_com = acos(mdotr)*RTOD-90;

% Focus of Expansion
% motion vector should be parallel to radial vector and therefore
% perpendicular to r_perp
mdotr_perp = dot(m,r_perp)';
residuals_foe = acos(mdotr_perp)*RTOD-90;

% limit the residuals using a simple M-estimator
if true
  MAX_ANGLE_com = 5.0*10^abs(y(3)); % degrees
  MAX_ANGLE_foe = MAX_ANGLE_com;
else
  MAX_ANGLE_com = prctile(abs(residuals_com),60);
  MAX_ANGLE_foe = prctile(abs(residuals_foe),60);
end
%disp([MAX_ANGLE_com,MAX_ANGLE_foe]);
[residuals_com,osel_com] = mestmc(residuals_com,y,MAX_ANGLE_com);
[residuals_foe,osel_foe] = mestmc(residuals_foe,y,MAX_ANGLE_foe);

% compute if motion is consistent which other inliers
[residuals_com,osel_com] = checkmotion(residuals_com,mdotr_perp,osel_com,MAX_ANGLE_com);
[residuals_foe,osel_foe] = checkmotion(residuals_foe,mdotr,osel_foe,MAX_ANGLE_foe);

% for FOE add error for distance away from inital guess
residuals_foe = residuals_foe + sqrt((y - y_o)'*(y - y_o));


error2_com = residuals_com'*residuals_com;
error2_foe = residuals_foe'*residuals_foe;
if error2_com < error2_foe
  residuals = residuals_com;
  FOEFLAG = false;
  osel = osel_com;
else
  % for FOE add error for distance away from inital guess
  %  residuals = residuals_foe + sqrt((y - y_o)'*(y - y_o));
  residuals = residuals_foe;
  FOEFLAG = true;
  osel = osel_foe;
end


%====================================================================================
function [residuals,osel] = checkmotion(residuals,dir,osel,MAX_ANGLE)
N  = length(dir);            % number of points
isel = setdiff([1:N],osel);  % inlier index
Ni = length(isel);           % number of inliers

% FOE check if inward or outward expansion
% COM check clockwise or counterclockwise
ii  = find(dir(isel) > 0);
ii = isel(ii);

if length(ii) >= Ni/2
  % in the case of FOE more than half of the direction vectors point
  % radially outward therefore, add additional cost to the vectors which are
  % oriented 180 from this direction.
  
  % in the case of COM more than half of the tangent vectors point in the
  % same direction as motion.  therefore, add additional cost to the vectors
  % which are orientad 180 from this direction.

  jj = setdiff(isel,ii);
  residuals(jj) = MAX_ANGLE*sign(residuals(jj));
  osel(end+1:end+length(jj)) = jj;
else
  residuals(ii) = MAX_ANGLE*sign(residuals(ii));
  osel(end+1:end+length(ii)) = ii;
end

%===========================================================================================
function [residuals,osel] = mestmc(residuals,y,THRESH);

% simple M-estimator limits max cost
osel = find(abs(residuals) >= THRESH); % find outliers

% limit the cost of outliers
%residuals_orig  = residuals;
residuals(osel) = sign(residuals(osel))*THRESH;

%disp('M-est residuals')
%[reshape(residuals_orig,1,[]); reshape(residuals,1,[])]
%fprintf('MAX_ANGLE = %.2e\ty(3) = %.2e\n',MAX_ANGLE,y(3));

