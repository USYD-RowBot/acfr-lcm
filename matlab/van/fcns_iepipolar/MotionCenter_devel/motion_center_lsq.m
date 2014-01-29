function [uc,vc,r] = motion_center_lsq(u1,v1,u2,v2)

% compose vectors
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% observation matrix is motion direction vector
%A = (x2 - x1)';
mvec = (x2-x1);
mvec = mvec./repmat(sqrt(dot(mvec,mvec)),[2 1]);
A = mvec';

% measurement vector
%b = dot((x2 - x1),0.5*(x2 + x1))';
b = dot(mvec,0.5*(x2 + x1))';

% center of motion
y = pinv(A)*b;
uc = y(1);
vc = y(2);

% residual
r = A*y-b;
