function [y] = motion_center_homo(u1,v1,u2,v2)

% vector representation
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% homogenous vector representation
X1 = homogenize(u1,v1);
X2 = homogenize(u2,v2);

% construct motion vector
m = (x2-x1);

% construct an orthogonal vector
m_perp = m([2,1],:);
m_perp(1,:) = -m_perp(1,:);


% compute midpoint
p1 = 0.5*(x2 + x1);

% compute a point which lies on vector orthogonal to motion vector
p2 = p1 + m_perp;

% homogenous representation
P1 = homogenize(p1(1,:)',p1(2,:)');
P2 = homogenize(p2(1,:)',p2(2,:)');

% equation of line that contains these two points & which is
% orthogonal to motion vector
L = cross(P1,P2);

% find the point y which lies on all of these lines
% y' * L = 0  ==>  L' * y = 0
[U,S,V] = svd(L');

% solution is the unit vector associated with the smallest singular value
y = V(:,end)
