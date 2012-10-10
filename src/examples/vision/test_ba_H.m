function [varargout] = test_ba_H (t_o,rph_o,n_o,d_o,u1_o,v1_o,u1,v1,u2,v2,K)
%TWOVIEW_BUNDLEADJUST_H two-view bundle adjustment of pose and plane parameter.
%  
%  twoview_BundleAdjust_H(t_o,rph_o,n_o,d,u1,v1,u2,v2,K) estimates the
%  relative pose parameters based upon an initial guess of the plane
%  parameter. n_o: plane normal d = dist to plane n^T X+d =0 (plane equ)
%  Euler rotation angles rph_0 and the [3 x 1] baseline vector t_0. [N x 1]
%  vectors u1, v1, u2, v2 are the measured image points.  The output
%  relative pose vector p_est is a [5 x 1] vector composed of
%  p_est = [azimuth, elevation, euler_roll, euler_pitch, euler_heading]' 
%  where the baseline is constrained to a vector of unit magnitude.
%  
%  Note that rph_o and t_o are defined such that the camera projection
%  matrices are of the form: P1 = [I | 0]   P2 = [R | t].
%
%  OUTPUT 1: (p_est,Cov_pest,exitflag,output)
%  OUTPUT 3: (p_est,Cov_pest,Cov_full,exitflag,output)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2009/02/17      ak          Created from twoview_BundleAdjust_Rae.m

error(nargchk(11,11,nargin));
error(nargoutchk(4,6,nargout));

%=========================================
% SPARSE LEVENBERG-MARQUARDT MINIMIZATION
%=========================================
% determine azimuth, elevation baseline representation
b_o = trans2dm(t_o);
azim_o = b_o(1);
elev_o = b_o(2);  
nb_o = trans2dm(n_o); %force it to be colume vector

% stack homography elements + u_hat into a parameter vector
% u_hat is the true image pixel to be estimated (8+2nx1)
u_tmp = zeros(2*length(u1_o),1);
u_tmp(1:2:end) = u1_o;
u_tmp(2:2:end) = v1_o;
p_o = [azim_o; elev_o; rph_o; nb_o(1); nb_o(2); d_o; u_tmp];

%----------------------------------------------------------------------
% calculate the Jacobian sparsity pattern for
% cost_vector = [(u1i-u1i_true); ...
%	             (v1i-v1i_true); ...
%	             (u2i-u2i_true); ...
%	             (v2i-v2i_true)];
Npoints = length(u1);
% block partial w.r.t. homography 
JBlockh  = [zeros(2,8); ... 
	        ones(2,8)];     
% block partial w.r.t. image point
JBlockImgpt = [eye(2); ones(2,2);];

Jpattern = spalloc(4*Npoints,8+2*Npoints,Npoints*(nnz(JBlockh)+nnz(JBlockImgpt)));
i = 1; j = 1;
for k=1:Npoints
  Jpattern(i:(i+3),1:8) = JBlockh;        % partial w.r.t. pose
  Jpattern(i:(i+3),8+(j:j+1)) = JBlockImgpt; % partial w.r.t. scene
  i = i+4;
  j = j+2;
end

%----------------------------------------------------------------------

% define optimization settings
Options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LargeScale','on', ...         % use large-scale algorithm
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
           'JacobPattern',Jpattern, ...   % jacobian sparsity pattern
           'Jacobian','off', ...          % jacobian function defined by user
		   'MaxFunEvals',1e8, ...         % let it iterate indefinitely
		   'MaxIter',100, ...             % if initialized correctly, should coverge very quickly (was5e8)
		   'Display','off');              % level of display

       		   
% perform nonlinear minimization
[p_est,resnorm,residuals,exitflag,output,lambda,J] = ...
    lsqnonlin(@reprojection_error,p_o,[],[],Options,u1,v1,u2,v2,K);

%----------------------------------------------------------------------

% optimized pose parameters
p_est = p_est(1:5);

% calculate covariance of MLE pose estimate under the assumption
% that measured feature points have an isotropic Gaussian covariance of
% Cov_uv = diag([1 1])
Cov_pest = pose_covariance(J);

try
    W = sparse(chol(Cov_pest));
    Cov_pest=Cov_pest(1:5,1:5);
catch
    Cov_pest = [];
end
% Cov_pest = J;

%----------------------------------------------------------------------

%=============================
% OUTPUT RESULTS
%=============================
switch nargout
 case 4 %p_est,Cov_pest,exitflag,output
  varargout{1} = p_est;
  varargout{2} = Cov_pest;
  varargout{3} = exitflag;
  varargout{4} = output;
 case 5 %p_est,Cov_pest,Cov_full,exitflag,output
  varargout{1} = p_est;
  varargout{2} = Cov_pest;
  varargout{3} = full_covariance(J);
  varargout{4} = exitflag;
  varargout{5} = output;  
 otherwise error('Incorrect number of outputs');
end


%******************************************************************************
function cost_vector = reprojection_error(p,u1,v1,u2,v2,K)
Npoints = length(u1);

% extract individual elements from parameter vector
t = dm2trans([p(1); p(2); 1]);
rph = p(3:5);
R=rotxyz(rph);
n = dm2trans([p(6); p(7); 1]);
d = p(8);
U1_hat = reshape(p(9:end),[2 Npoints]);
u1_hat = U1_hat(1,:)';
v1_hat = U1_hat(2,:)';

% build homography H
% H = K(R-tn'./d)inv(K)
H=K*(R-t*n'./d)*inv(K);

% calculate U2_hat
U2_hat = H*homogenize(u1_hat,v1_hat);
[u2_hat,v2_hat] = dehomogenize(U2_hat);

% calculate the reprojection error noting that matlab expects the cost
% vector to be the non-squared error terms
% stack for the error vector
% [ u1_hat - u1
%   v1_hat - v1
%   u2_hat - u2
%   v2_hat - v2 ]    4n x 1 vector

cost_vector = zeros(4*Npoints,1);
cost_vector(1:4:4*Npoints) = (u1_hat-u1);
cost_vector(2:4:4*Npoints) = (v1_hat-v1);
cost_vector(3:4:4*Npoints) = (u2_hat-u2);
cost_vector(4:4:4*Npoints) = (v2_hat-v2);

%******************************************************************************
function Cov_a = pose_covariance(J)
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
Cov_a = zeros(8);
for k=1:Npoints % for each 4-tuple measurement of reprojection error
  % partial w.r.t. pose
  Ai = full(J(i:(i+3),1:8));         % [4 x 8]
  % partial w.r.t. scene point Xi
  Bi = full(J((i):(i+3),8+(j:j+1))); % [4 x 4]
  % intermediate terms
  Ui = Ai'*Ai;                       % [8 x 8]
  Vi = Bi'*Bi;                       % [4 x 4]
  Wi = Ai'*Bi;                       % [8 x 4]
  Yi = Wi*inv(Vi);                   % [8 x 4]

  Zi = Yi*Wi';                       % Zi = Yi*Wi' = Wi*inv(Vi)*W', but may not be sym
  Zi = (Zi+Zi')/2;                   % force Zi to be symmetric matrix
                                     
  % pose information matrix
  Cov_a = Cov_a + (Ui - Zi);         % [8 x 8]
  % increment indices
  i = i+4; % step J row index to next 4-tuple of reprojection error
  j = j+2; % step J column index to next scene point Xi
end
Cov_a = pinv(Cov_a);

%******************************************************************************
function Cov_full = full_covariance(J)
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
Y = zeros(5,3*Npoints);
Cov_a = zeros(5);
for k=1:Npoints % for each 4-tuple measurement of reprojection error
  % partial w.r.t. pose
  Ai = full(J(i:(i+3),1:5));         % [4 x 6]
  % partial w.r.t. scene point Xi
  Bi = full(J(i:(i+3),5+[j:(j+2)])); % [4 x 3]
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
Cov_a = pinv(Cov_a);

% now that we have the pose covariance and intermediate expressions, loop
% through and fill in the full covariance matrix (i.e. covariance of pose
% and scene).  note that we only have to calculate the upper half of the
% full covariance matrix since the lower half is related by the transpose.
Cov_full = zeros(5+3*Npoints,5+3*Npoints);
Cov_full(1:5,1:5) = Cov_a;
Cov_full(1:5,6:end) = -Cov_a*Y;
for i=1:3:(3*Npoints)
  for j=i:3:(3*Npoints)
    Yi = Y(:,i:(i+2));
    Yj = Y(:,j:(j+2));
    Cov_bibj = Yi'*Cov_a*Yj;
    if i==j
      Vi = V(:,i:(i+2));
      Cov_bibj = Cov_bibj + inv(Vi);
    end
    Cov_full(5+[i:(i+2)],5+[j:(j+2)]) = Cov_bibj;
  end
end
% fill in the rest of the symmetric matrix 
Cov_full = (Cov_full + Cov_full') - diag(diag(Cov_full));
