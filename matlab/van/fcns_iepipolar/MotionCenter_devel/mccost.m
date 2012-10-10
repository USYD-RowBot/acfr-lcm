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
[residuals_com,osel_com,MAX_ANGLE_com] = mestmc(residuals_com,y);
[residuals_foe,osel_foe,MAX_ANGLE_foe] = mestmc(residuals_foe,y);

% compute if motion is consistent which other inliers
[residuals_com,osel_com] = checkmotion(residuals_com,mdotr_perp,osel_com,MAX_ANGLE_com);
[residuals_foe,osel_foe] = checkmotion(residuals_foe,mdotr,osel_foe,MAX_ANGLE_foe);

error2_com = residuals_com'*residuals_com;
error2_foe = residuals_foe'*residuals_foe;
if error2_com < error2_foe
  residuals = residuals_com;
  FOEFLAG = false;
  osel = osel_com;
else
  % for FOE add error for distance away from inital guess
  residuals = residuals_foe + sqrt((y - y_o)'*(y - y_o));
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

