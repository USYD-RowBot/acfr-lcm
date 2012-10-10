function [y,FOEFLAG] = motioncenter_lsq(u1,v1,u2,v2)
%function [y,FOEFLAG] = motioncenter_lsq(u1,v1,u2,v2)

N = length(u1);

% homogenous vector representation
U = homogenize([u1;u2],[v1;v2]);

% normalize point cloud by scaling and translating so that centroid is at
% origin and average distance is sqrt(2)
[T,Un] = normalize_2d_pts(U);

[u1n,v1n] = dehomogenize(Un(:,1:N));
[u2n,v2n] = dehomogenize(Un(:,N+1:end));

% vector representation
x1 = [u1n'; v1n'];
x2 = [u2n'; v2n'];

% compute the solution associated with calculating the focus of expansion
[yn_foe,residual_foe] = foe(x1,x2);
error2_foe = residual_foe'*residual_foe;

% compute the solution associated with calculating the center of motion
[yn_com,residual_com] = com(x1,x2);
error2_com = residual_com'*residual_com;

% magnitude of FOE vector
mag_foe = yn_foe/yn_foe(3);
mag_foe = sqrt(mag_foe'*mag_foe);

if mag_foe < 10 && ... % check that FOE is *within* image (finite)
   error2_foe < error2_com
  yn = yn_foe;
  FOEFLAG = true;
else
  yn = yn_com;
  FOEFLAG = false;
end

% transform solution back to the original solution space
y = inv(T)*yn;


%============================================================================
function [y,residual] = foe(x1,x2)
% FOCUS OF EXPANSION
% If the relative camera motion is mostly along the Z-axis,
% then the vector field points towards the focus of expansion.
  
% homogenous represenation
X1 = homogenize(x1(1,:)',x1(2,:)');
X2 = homogenize(x2(1,:)',x2(2,:)');

% equation of line containing the two endpoints
L = cross(X1,X2);
% normalize each line equation to unit magnitude
L = L./repmat(sqrt(dot(L,L)),[3 1]);

% solve for the homogenous point y which is common to all of these lines
% i.e.  y'*L = 0  with constraint sqrt(y'*y) = 1
% the solution is the vector associated with the smallest singular value
[U,S,V] = svd(L',0);
y = V(:,end);

residual = L'*y;

%============================================================================
function [y,residual] = com(x1,x2)
% CENTER OF MOTION
% If the relative camera motion is mostly rotational or X,Y translational,
% then the vector field "spins" about an axis in the image plane dubbed
% the center of motion.

% construct motion vector
m = (x2-x1);
% normalize each vector to unit magnitude
m = m./repmat(sqrt(dot(m,m)),[2 1]);

% construct a vector orthogonal to motion vector
P = [0 -1; 1 0];
m_perp = P*m;

% compute midpoint of motion vector
p = 0.5*(x2 + x1);

% compute a point which lies on the vector emanating from the midpoint and 
% is orthogonal to motion vector
q = p + m_perp;

% homogenous representation
P = homogenize(p(1,:)',p(2,:)');
Q = homogenize(q(1,:)',q(2,:)');

% equation of line that contains these two points & which is
% orthogonal to motion vector
L = cross(P,Q);
% normalize each line equation to unit magnitude
L = L./repmat(sqrt(dot(L,L)),[3 1]);

% solve for the homogenous point y which is common to all of these lines
% i.e.  y'*L = 0  with constraint sqrt(y'*y) = 1
% the solution is the vector associated with the smallest singular value
[U,S,V] = svd(L',0);
y = V(:,end);

residual = L'*y;
