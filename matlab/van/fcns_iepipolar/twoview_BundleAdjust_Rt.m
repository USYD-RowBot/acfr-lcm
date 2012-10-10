function [varargout] =  twoview_BundleAdjust_Rt(t_o,rph_o,X_o,u1,v1,u2,v2,K,f_robust)
%TWOVIEW_BUNDLEADJUST_RT two-view bundle adjustment of pose and scene.
%  
%  twoview_BundleAdjust_Rt(t_0,rph_0,X_0,u1,v1,u2,v2,K) estimates the
%  relative pose parameters based upon an initial guess of the [3 x 1] XYZ
%  Euler rotation angles rph_0 and the [3 x 1] baseline vector t_0.  X_o is
%  a [3 x N] array of triangulated 3D scene feature points while the [N x 1]
%  vectors u1, v1, u2, v2 are the measured image points.  The output
%  relative pose vector p_est is a [6 x 1] vector composed of
%  p_est = [tx, ty, tz, euler_roll, euler_pitch, euler_heading]' 
%  where t is a direction vector of unit magnitude.
%
%  f_robust allows the user to provide a pointer to a robust weighting function
%  that modifies and returns the cost vector used in the LM optimization
%  cost_vec_weighted = f_robust(cost_vec);
%
%  Note that rph_o and t_o are defined such that the camera projection
%  matrices are of the form: P1 = [I | 0]   P2 = [R | t].
%
%  OUTPUT 1: (p_est,Cov_pest,exitflag,output)
%  OUTPUT 2: (p_est,Cov_pest,X_est,exitflag,output)
%  OUTPUT 3: (p_est,Cov_pest,X_est,Cov_full,exitflag,output)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09/22/2003      rme         Created from estim_thatrph_gs.m
%    09/25/2003      rme         Finished implementing the sparse LM
%                                estimation and calculation of parameter
%                                covariance.
%    10/23/2003      rme         Normalize both baseline and scene prior
%                                to optimization.  Also, modified
%                                covariance calculations to use Jt to be
%                                consistent with other functions.
%    11-08-2004      rme         Modified calculation of Jpattern to be more efficient.
%    12-30-2010      ncb         Added option for robust cost weighting function.  

error(nargchk(8,9,nargin));
error(nargoutchk(4,6,nargout));

if nargin==8
    f_robust = [];    
end

%========================================
% SPARSE LEVENBERG-MARQUARDT MINIMIZATION
%========================================
% normalize baseline and scene
t_o = t_o/norm(t_o);
X_o = X_o/norm(t_o);

% stack pose elements into a parameter vector
p_o = [t_o; rph_o; X_o(:)];

%----------------------------------------------------------------------
% calculate the Jacobian sparsity pattern for
% P1 = [I | 0]   P2 = [R | t] and error terms of the following form:
% cost_vector = [(u1i-u1i_true); ...
%	         (v1i-v1i_true); ...
%	         (u2i-u2i_true); ...
%	         (v2i-v2i_true)];
Npoints = length(u1);
% block partial w.r.t. pose  
JBlockPose  = [zeros(2,6); ... %(u1i_true,v1i_true) = P1*Xi, i.e. not a fcn of R,t
	       1 0 1 1 1 1; ...% u2i_true not a fcn of t_y
	       0 1 1 1 1 1];   % v2i_true not a fcn of t_x
% block partial w.r.t. scene
JBlockScene = [1 0 1; ...      % u1i_true fcn of Xi_x & Xi_z
	       0 1 1; ...      % v1i_true fcn of Xi_y & Xi_z
	       ones(2,3)];     % (u2i_true,v2i_true) fcn of Xi

Jpattern = spalloc(4*Npoints,6+3*Npoints,Npoints*(nnz(JBlockPose)+nnz(JBlockScene)));
i = 1; j = 1;
for k=1:Npoints
  Jpattern(i:(i+3),1:6) = JBlockPose;        % partial w.r.t. pose
  Jpattern(i:(i+3),6+[j:j+2]) = JBlockScene; % partial w.r.t. scene
  i = i+4;
  j = j+3;
end

%----------------------------------------------------------------------
% define optimization settings
Options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LargeScale','on', ...         % use large-scale algorithm
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
		   'Jacobian','off', ...          % jacobian function defined by user
		   'JacobPattern',Jpattern, ...   % jacobian sparsity pattern
		   'MaxFunEvals',1e6, ...         % let it iterate indefinitely
		   'MaxIter',200, ...             % if initialized correctly should coverge very quickly
		   'Display','off');              % level of display

% perform nonlinear minimization
[p_est,resnorm,residuals,exitflag,output,lambda,J] = ...
    lsqnonlin(@reprojection_error,p_o,[],[],Options,u1,v1,u2,v2,K,f_robust);

%----------------------------------------------------------------------
% optimized feature points
X_est = reshape(p_est(7:end),[3 Npoints]);

% optimized pose parameters
p_est = p_est(1:6);

% the baseline direction vector (t_est) was not explictly constrained to
% have unit magnitude during the minimization.  normalize the baseline and
% scene and update the 1st order covariance accordingly.
t_est = p_est(1:3); 
tmag = norm(t_est);
[t_hat,Jt] = unitize(t_est);
p_est = [t_hat; p_est(4:6)];
X_est = X_est/tmag;
%----------------------------------------------------------------------

%=============================
% OUTPUT RESULTS
%=============================
switch nargout
 case 4 %p_est,Cov_pest,exitflag,output
  % calculate covariance of ML pose estimate under the assumption
  % that measured feature points have an isotropic Gaussian covariance of
  % Cov_uv = diag([1 1])
  varargout{1} = p_est;
  varargout{2} = pose_covariance(J,Jt);
  varargout{3} = exitflag;
  varargout{4} = output;
 case 5 %p_est,Cov_pest,X_est,exitflag,output
  % calculate covariance of ML pose estimate under the assumption
  % that measured feature points have an isotropic Gaussian covariance of
  % Cov_uv = diag([1 1])
  varargout{1} = p_est;
  varargout{2} = pose_covariance(J,Jt);
  varargout{3} = X_est;
  varargout{4} = exitflag;
  varargout{5} = output;
 case 6 %p_est,Cov_pest,X_est,Cov_full,exitflag,output
  % calculate covariance of ML pose/scene estimate under the assumption
  % that measured feature points have an isotropic Gaussian covariance of
  % Cov_uv = diag([1 1])  
  varargout{1} = p_est;
  varargout{2} = pose_covariance(J,Jt);  
  varargout{3} = X_est;
  varargout{4} = full_covariance(J,Jt,tmag);
  varargout{5} = exitflag;
  varargout{6} = output;  
 otherwise error('Incorrect number of outputs');
end


%******************************************************************************
function cost_vector = reprojection_error(p,u1,v1,u2,v2,K,f_robust)
Npoints = length(u1);

% extract individual elements from parameter vector
t = p(1:3);
rph = p(4:6);
X = reshape(p(7:end),[3 Npoints]);

% compose camera projection matrices
P1 = K*[eye(3), zeros(3,1)];
P2 = K*[rotxyz(rph), t];

% project scene points into each camera
% note that (X,Y,Z,1)' does not consider points at infinity which
% shouldn't be a problem for our typical bottom-looking camera configuration
U1_true = P1*[X; ones(1,Npoints)];
U2_true = P2*[X; ones(1,Npoints)];

[u1_true,v1_true] = dehomogenize(U1_true);
[u2_true,v2_true] = dehomogenize(U2_true);

% calculate the reprojection error noting that matlab expects the cost
% vector to be the non-squared error terms
% cost_vector = [(u1i-u1i_true); ...
%	         (v1i-v1i_true); ...
%	         (u2i-u2i_true); ...
%	         (v2i-v2i_true)];
cost_vector = zeros(4*Npoints,1);
cost_vector(1:4:4*Npoints) = (u1-u1_true);
cost_vector(2:4:4*Npoints) = (v1-v1_true);
cost_vector(3:4:4*Npoints) = (u2-u2_true);
cost_vector(4:4:4*Npoints) = (v2-v2_true);

% weight the cost vector based on the robust cost function if avaliable
if ~isempty(f_robust)
    cost_vector = f_robust(cost_vector);
end


%******************************************************************************
function Cov_a = pose_covariance(J,Jt)
% calculate the pose covariance by exploiting the sparse block structure of
% the Jacobian.  the notation and algorithm follows HZ A.4.3 pg 577 sparse
% LM covariance calculation.  
  
% the Jacobian J has a sparse block structure of the form
% J = [A1 | B1
%      A2 |   B2
%      A3 |     B3
%       :          \
%      AN |          BN];
Npoints = size(J,1)/4;
i = 1;
j = 1;
Cov_a = zeros(6);
for k=1:Npoints % for each 4-tuple measurement of reprojection error
  % partial w.r.t. pose
  Ai = full(J(i:(i+3),1:6));         % [4 x 6]
  % partial w.r.t. scene point Xi
  Bi = full(J(i:(i+3),6+[j:(j+2)])); % [4 x 3]
  % intermediate terms
  Ui = Ai'*Ai;                       % [6 x 6]
  Vi = Bi'*Bi;                       % [3 x 3]
  Wi = Ai'*Bi;                       % [6 x 3]
  Yi = Wi*inv(Vi);                   % [6 x 3]
  % pose information matrix
  Cov_a = Cov_a + (Ui - Yi*Wi');     % [6 x 6]
  % increment indices
  i = i+4; % step J row index to next 4-tuple of reprojection error
  j = j+3; % step J column index to next scene point Xi
end

% even though the pose vector is parameterized as p = [t; rph], p really
% only has 5 DOF since t is just a direction vector.  therefore, we
% enforce the contraint that p's covariance matrix be singular by setting
% the smallest singular value to zero.
%[UU,DD,VV] = svd(Cov_a);
%DD(6,6) = 0;
%Cov_a = VV*pinv(DD)*UU';
JJt = [Jt,       zeros(3); ...
       zeros(3), eye(3)];
Cov_a = JJt*pinv(Cov_a)*JJt';

%******************************************************************************
function Cov_full = full_covariance(J,Jt,tmag)
% calculate the full covariance (pose and scene) by exploiting the sparse
% block structure of the Jacobian.  the notation and algorithm follows HZ
% A.4.3 pg 577 sparse LM covariance calculation.
  
% the Jacobian J has a sparse block structure of the form
% J = [A1 | B1
%      A2 |   B2
%      A3 |     B3
%       :          \
%      AN |          BN];
Npoints = size(J,1)/4;
i = 1;
j = 1;
V = zeros(3,3*Npoints);
Y = zeros(6,3*Npoints);
Cov_a = zeros(6);
for k=1:Npoints % for each 4-tuple measurement of reprojection error
  % partial w.r.t. pose
  Ai = full(J(i:(i+3),1:6));         % [4 x 6]
  % partial w.r.t. scene point Xi
  Bi = full(J(i:(i+3),6+[j:(j+2)])); % [4 x 3]
  % intermediate terms
  Ui = Ai'*Ai;                       % [6 x 6]
  Vi = Bi'*Bi;                       % [3 x 3]
  Wi = Ai'*Bi;                       % [6 x 3]
  Yi = Wi*inv(Vi);                   % [6 x 3]
  % pose information matrix
  Cov_a = Cov_a + (Ui - Yi*Wi');     % [6 x 6]
  % store Vi,Yi which are required for calculation of scene covariance
  V(:,j:(j+2)) = Vi;
  Y(:,j:(j+2)) = Yi;
  % increment indices
  i = i+4; % step J row index to next 4-tuple of reprojection error
  j = j+3; % step J column index to next scene point Xi
end

% even though the pose vector is parameterized as p = [t; rph], p really
% only has 5 DOF since t is just a direction vector.  therefore, we
% enforce the contraint that p's covariance matrix be singular by setting
% the smallest singular value to zero.
%[UU,DD,VV] = svd(Cov_a);
%DD(6,6) = 0;
%Cov_a = VV*pinv(DD)*UU';
JJt = [Jt,       zeros(3); ...
       zeros(3), eye(3)];
Cov_a = JJt*pinv(Cov_a)*JJt';

% now that we have the pose covariance and intermediate expressions, loop
% through and fill in the full covariance matrix (i.e. covariance of pose
% and scene).  note that we only have to calculate the upper half of the
% full covariance matrix since the lower half is related by the transpose.
% recall that the baseline and scene are normalized to baseline unit
% magnitude, therefore the need to normalize the scene covariance by tmag.
Cov_full = zeros(6+3*Npoints,6+3*Npoints);
Cov_full(1:6,1:6) = Cov_a;
Cov_full(1:6,7:end) = -Cov_a*Y/tmag;
for i=1:3:(3*Npoints)
  for j=i:3:(3*Npoints)
    Yi = Y(:,i:(i+2));
    Yj = Y(:,j:(j+2));
    Cov_bibj = Yi'*Cov_a*Yj;
    if i==j
      Vi = V(:,i:(i+2));
      Cov_bibj = Cov_bibj + inv(Vi);
    end
    Cov_full(6+[i:(i+2)],6+[j:(j+2)]) = Cov_bibj/tmag^2;
  end
end
% fill in the rest of the symmetric matrix 
Cov_full = (Cov_full + Cov_full') - diag(diag(Cov_full));
