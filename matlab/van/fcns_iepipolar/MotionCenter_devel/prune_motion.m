function [sel,theta] = prune_motion(u1,v1,u2,v2,uc,vc)
N = length(u1);
  
% compose vector
x1 = [u1'; v1'];
x2 = [u2'; v2'];
xc = [uc; vc];

% motion unit direction vector
mvec = x2-x1;
mvec = mvec./repmat(sqrt(dot(mvec,mvec)),[2 1]);

% compute radial vector from motion center to midpoint
rvec = 0.5*(x1+x2) - repmat(xc,[1 N]);

% compute tangent unit vector
tvec = rvec([2,1],:);
tvec(1,:) = -tvec(1,:);
tvec = tvec./repmat(sqrt(dot(tvec,tvec)),[2 1]);

% compute angle between motion vector and tangent vector
theta = acos(dot(mvec,tvec));

sel = [];

