function [R,t,E] = relorient_horn(x1,y1,x2,y2,R)
%RELORIENT_HORN uses the coplanarity constraint to solve for the
%               best fit baseline direction and orientation.
%
%   [R,t,E] = RELORIENT_HORN(X1,Y1,X2,Y2,R) returns the [3 x 1] baseline
%   direction t given the normalized image coordinates (X1,Y1) and (X2,Y2)
%   (each are [N x 1] vectors) and intial orientation guess R ([3 x 3]
%   orthonormal rotation matrix).  R and t are defined such that the camera
%   projection matrices are of the form: P1 = [I | 0]   P2 = [R | t]
%   E is the coplanarity error cost associated with R,t and is an
%   optional output argument.
%
%   This algorithm is based upon:
%   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created.
%    09-11-2003      rme         Ironed out the bugs and vectorized the
%                                "for loop" operations to optimize the
%                                computation for Matlab.  The code runs
%                                very fast now.
%    09-20-2003      rme         Updated to store scalar part of
%                                quaternion as first element instead of
%                                as last element.
%    09-22-2003                  Fixed error in normalizing scene ray
%                                vectors.  I was dividing by the dot
%                                product and not the square-root of the
%                                dot product.

error(nargchk(5,5,nargin));  

MIN_ITERATIONS = 10;
MAX_ITERATIONS = 50;

% number of features
Nf = length(x1);

% scene ray vectors
r_l = [x1, y1, ones(Nf,1)]'; % left camera
r_r = [x2, y2, ones(Nf,1)]'; % right camera

% normalize scene ray vectors to unit direction vectors
r_l = r_l./repmat(sqrt(dot(r_l,r_l)),[3 1]); % normalized to unit vector
r_r = r_r./repmat(sqrt(dot(r_r,r_r)),[3 1]); % normalized to unit vector

% solve for a least-squares best fit baseline
% by enforcing the coplanarity constraint
t = lsq_baseline_direction(r_l,r_r,R);

% note that Horn defines the baseline direction in the
% opposite sense than I do
b = -t;

iterations = 0;
delta_b = [1 1 1]'; 
delta_w = delta_b;
Eprev = 1e4; 
E = Eprev-1;
while (iterations < MAX_ITERATIONS)
  %============================================
  % ITERATIVE UPDATE TO BASELINE DIRECTION
  % AND ORIENTATION
  %============================================
  % left camera represented in right coordinate frame
  % based upon current estimate of orientation
  r_lprime = R*r_l;
    
  % define the follwing products, following Horn's notation.
  % *note* i'm vectorizing the "for loop" to calculate an
  % equivalent expression but much much faster
  %-------------------------------------------------------
  %>>>>>>>>>>>>>> old way <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  %c_bar = 0;
  %d_bar = 0;
  %B = eye(3) - b*b';
  %C = zeros(3);
  %D = zeros(3);
  %Eprev = E;
  %E = 0;
  %F = zeros(3);
  %for i=1:Nf
  %  c = cross(r_lprime(:,i),r_r(:,i));
  %  t = b'*c;
  %  d = cross(r_lprime(:,i), cross(r_r(:,i),b));
  %  c_bar = c_bar + t*c;
  %  d_bar = d_bar + t*d;
  %  C = C + c*c';
  %  D = D + d*d';
  %  E = E + (t + c'*delta_b + d'*delta_w)^2;
  %  F = F + c*d';
  %end
  %-------------------------------------------------------
  %>>>>>>>>>>>>>>>>>>> vectorized <<<<<<<<<<<<<<<<<<<<<<<<
  c = cross(r_lprime,r_r);
  t = b'*c;
  d = cross(r_lprime, cross(r_r,repmat(b,[1 Nf])));
  c_bar = (t*c')';
  d_bar = (t*d')';
  C = c*c';
  B = eye(3)-b*b';
  D = d*d';
  F = c*d';
  %-------------------------------------------------------
  
  % form system of equations Hx = g where
  % x = [delta_b; delta_w];
  H = [B*C B*F; ...
       F'   D];
  g = -[B*c_bar; d_bar];

  % B is singular (b is an eigenvector with zero eigenvalue), thus the first
  % three equations in the system above are not independent.  one of them
  % will have to be removed.  for best numerical accuracy we eliminate the
  % equation with the smallest coefficients and replace it with the linear
  % constraint b'*delta_b = 0
  [tmp,row] = min(sum(abs(H(1:3,:)),2));
  H(row,:) = [b' 0 0 0];
  g(row) = 0;

  % solve the system of equations for the incremental update
  x = inv(H)*g;
  delta_b = x(1:3);
  delta_w = x(4:6);

  % calculate the coplanarity cost function and decide
  % whether to terminate or continue
  % *note* that the error expression is vectorized
  Eprev = E;
  E = (t' + c'*delta_b + d'*delta_w);
  E = E'*E;
  if (iterations > MIN_ITERATIONS) && (E > Eprev)
    % terminate iteration if error has increased
    break;
  else
    % adjust the baseline
    b = b + delta_b; % apply infitesimal baseline update
    b = b/norm(b);   % normalize to a unit direction vector

    % adjust the orientation using quaternions. q is the quaternion
    % associated with the infinitesimal rotation delta_w
    q = [1; 0.5*delta_w]; %>>>>> note the 0.5 coefficent on    <<<<<
			  %>>>>> delta_w, Horn is missing this <<<<<
                          %>>>>> in his paper!!!               <<<<<
    q = q/norm(q);   % normalize to unit quaternion
    Q = quat2rot(q); % convert to a rotation matrix
    R = Q*R;         % apply infitesimal rotation update
    
    iterations = iterations + 1;
  end
  
end % while (iterations < max_iterations)

%fprintf('%s: computed R,t in %d iterations with residual error of %.2e\n', ...
%	mfilename,iterations,E);

% recompute the least-squares baseline based upon our final orientation
t = lsq_baseline_direction(r_l,r_r,R);
