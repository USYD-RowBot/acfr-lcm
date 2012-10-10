function [t,E] = lsq_baseline_direction(varargin)
%LSQ_BASELINE_DIRECTION uses the coplanarity constraint to solve for the
%                       best fit baseline direction in a least-squares sense.
%
%   [T,E] = LSQ_BASELINE_DIRECTION(X1,Y1,X2,Y2,R) returns the [3 x 1]
%   baseline direction T given the normalized image coordinates (X1,Y1)
%   (X2,Y2) ([N x 1] vectors) and intial orientation guess R ([3 x 3]
%   orthonormal rotation matrix).  E is the coplanarity error cost
%   associated with R,t and is an optional output argument.
%
%   [T,E] = LSQ_BASELINE_DIRECTION(r_1,r_2,R) does the same, but uses the
%   normalized unit direction ray vectors r_1 and r_2 where each is a 
%   [3 x N] matrix.
%
%   R and t are defined such that the camera projection matrices are of the
%   form: P1 = [I | 0]    P2 = [R | t]
%
%
%   This algorithm is based upon:
%   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created.
%    09-11-2003      rme         Vectorized the "for loop" operation
%                                gaining a huge speed-up in Matlab
%    09-22-2003                  Fixed error in normalizing scene ray
%                                vectors.  I was dividing by the dot
%                                product and not the square-root of the
%                                dot product.
  
error(nargchk(3,5,nargin));
  
if nargin == 3
  r_l = varargin{1};
  r_r = varargin{2};
  R   = varargin{3};
else
  % scene ray vectors
  r_l = [varargin{1}, varargin{2}, ones(Nf,1)]'; % left camera
  r_r = [varargin{3}, varargin{4}, ones(Nf,1)]'; % right camera

  % normalize scene ray vectors to unit direction vectors
  r_l = r_l./repmat(sqrt(dot(r_l,r_l)),[3 1]); % normalized to unit vector
  r_r = r_r./repmat(sqrt(dot(r_r,r_r)),[3 1]); % normalized to unit vector
end
  
% number of features
Nf = size(r_l,2);  
  
%============================================
% LEAST SQUARES SOLUTION FOR THE BASELINE
%============================================
% left camera represented in right coordinate frame
% based upon current estimate of orientation
r_lprime = R*r_l;

% compute the sum of baseline error cost function,
% follwing Horn's notation.
% *note* i'm vectorizing the "for loop" to calculate an
% equivalent expression but much much faster
%------------------------------------------------------
%>>>>>>>>>>>> old way <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%c = [];
%C = zeros(3);
%for i=1:Nf
%  ci = cross(r_lprime(:,i),r_r(:,i));
%  C = C + ci*ci';
%  c = [c, ci];
%end
%------------------------------------------------------
%>>>>>>>>>>>>>>> vectorized <<<<<<<<<<<<<<<<<<<<<<<<<<<
c = cross(r_lprime,r_r);
C = c*c';
%------------------------------------------------------

% the coplanarity error cost we are trying to minimize is
% E = b'*C*b  with constraint b'*b = 1
% the solution for the baseline direction is the eigenvector
% associated with the smallest eigenvalue
[V,D] = eig(C);
b = V(:,1);

% E is the error cost associated with b
E = b'*C*b;

% the above solution for b has unit magnitude, but may have the wrong
% sign, i.e. the correct solution is -b
% test the current value of b to see if a majority of points lie
% in *front* of the two cameras
% note: we are only interested in the *signs* of alpha and beta, therefore
% dividing by the magnitude of the cross-product is unnecessary
%alpha = dot(cross(repmat(b,[1 Nf]),r_r),c) ./ dot(c,c);
%beta  = dot(cross(repmat(b,[1 Nf]),r_lprime),c) ./ dot(c,c);
alpha = dot(cross(repmat(b,[1 Nf]),r_r),c);
beta  = dot(cross(repmat(b,[1 Nf]),r_lprime),c);
if (sum(alpha>0) < Nf/2) || (sum(beta>0) < Nf/2)
  % we have the wrong sign for the baseline direction, fix it
  b = -b;
  %disp('fixed wrong sign for baseline');  
end

% note that I define the baseline direction t in the
% opposite sense than Horn does
t = -b;
