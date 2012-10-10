function [yo] = motion_center_homo_lsq(u1o,v1o,u2o,v2o)

N = length(u1o);

% homogenous vector representation
Uo = homogenize([u1o;u2o],[v1o;v2o]);

[T,U_norm] = normalize_2d_pts(Uo);

[u1,v1] = dehomogenize(U_norm(:,1:N));
[u2,v2] = dehomogenize(U_norm(:,N+1:end));

% vector representation
x1 = [u1'; v1'];
x2 = [u2'; v2'];

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
y = V(:,end);

yo = inv(T)*y;
%yo = yo/yo(3);
