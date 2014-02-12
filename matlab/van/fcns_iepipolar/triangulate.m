function [X1,alpha,beta,gamma] = triangulate(R,t,u1,v1,u2,v2,K);
%TRIANGULATE find lest-squares scene structure.
%   X = TRIANGULATE(R,t,u1,v1,u2,v2,K) returns the [3 x N] matrix X
%   representing the scene structure as described in camera 1's
%   coordinate frame.  u1,v2 and u2,v2 are [N x 1] feature coordinates
%   measured in pixels.  K is the [3 x 3] camera calibration matrix.
%   The scene points X are defined as the lengths along camera rays r1
%   and r2 which minimizes the perpendicular distance between the ray
%   pairs.  Note that scene scale is measured in units of baseline magnitude.
%
%   [X,ALPHA,BETA,GAMMA] = TRIANGULATE(R,t,u1,v1,u2,v2,K) also returns
%   the [1 x N] vectors ALPHA, BETA, and GAMMA corresponding to the
%   length along rays r1 and r2 which minimize the perpendicular distance
%   and GAMMA is the corresponding minimal perpendicular distance.
%
%   R and t are defined such that the camera projection matrices are of the
%   form: P1 = K[I | 0]    P2 = K[R | t]
%
%   This algorithm is based upon:
%   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-22-2003      rme         Created and written.  This
%                                implementation is also inspired by Oscar
%                                Pizarro's triangulate.m

Np = length(u1);

% determine the Euclidian ray direction vectors for each image point
invK = inv(K);
r1 = invK*homogenize(u1,v1);
r2 = invK*homogenize(u2,v2);

% normalize the rays to unit magnitude
r1 = r1./repmat(sqrt(dot(r1,r1)),[3 1]);
r2 = r2./repmat(sqrt(dot(r2,r2)),[3 1]);

% rotate the rays from camera 1 into orientation of camera 2 frame
r1_prime = R*r1;

% following Horn's notation
b = -t;
c = cross(r1_prime,r2);
c_dot_c = dot(c,c);

% magnitudes along rays r1_prime and r2 where closest intersection occurs
alpha = dot(cross(repmat(b,[1 Np]),r2),c) ./ c_dot_c;
beta  = dot(cross(repmat(b,[1 Np]),r1_prime),c) ./ c_dot_c;

% perpendicular distance at closest intersection
gamma = dot(repmat(b,[1 Np]),c) ./ c_dot_c;

% scene as represented in camera frame 2 where scene features are defined
% to lie at midpoint of ray intersection
%X1_prime = repmat(alpha,[3 1]).*r1_prime + repmat(t,[1 Np]) + repmat(gamma,[3 1]).*c/2;
X2 = repmat(beta,[3 1]).*r2 - repmat(gamma,[3 1]).*c/2;

% scene as represented in camera frame 1
X1 = R'*(X2 - repmat(t,[1 Np]));
