function [u2,v2,Cov_u2v2] = relview_ptxfer(K,x_21,Z1,u1,v1,Cov_pose,Cov_Z1,Cov_u1v1)
%function [u2,v2,Cov_u2v2] = relview_ptxfer_sym(K,x_21,Z1,u1,v1,Cov_pose,Cov_Z1,Cov_u1v1)
% 
% returns the projected u1,v1 onto image 2 using pose info
% also covariance for each projection
% based on Ryan's transfer between absolute poses
% modified to work on relative poses
%
% Cov_u1v1 optional, default diag([2 2]);
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created from twoview_ptxfer.
%    08-31-2003      rme         Modified to be consistent with my code.
%    04-01-2004      rme         Modified to accept a *vector* of 
%                                scene depth and covariance Z1,Cov_Z1
%                                associated with feat pts (u1,v1)
%    01-03-2004      rme         Changed Cov_cell to Cov_u2v2 array.
  
if ~exist('Cov_u1v1','var') || isempty(Cov_u1v1);
  Cov_u1v1 = diag([2 2]); % assume 1.4 pixel sigma
end;

%======================================
% COMPUTE POINT TRANSFER BASED UPON 
% SCENE DEPTH AND RELATIVE POSE
%======================================
% predicted location of points (u1,v1) in image 2
[u2,v2] = ptxfer(x_21,u1,v1,Z1,K);

%========================================
% CALCULATE JACOBIAN OF TRANSFERED POINTS
%========================================
% jacobian w.r.t relative pose x_21
[Ju_p,Jv_p] = jacobian_wrt_pose(x_21,u1,v1,Z1,K);

% jacobian w.r.t. feature locations (u1,v1,Z1)
[Ju_f,Jv_f] = jacobian_wrt_points(x_21,u1,v1,Z1,K);


%=========================================
% 1ST ORDER COVARIANCE OF POSE PARAMETERS
%=========================================
N = length(u1);
Cov_u2v2 = zeros(2,2,N);

% init param covariance
Cov_p = zeros(9);
Cov_p(1:6,1:6) = Cov_pose;
Cov_p(7:8,7:8) = Cov_u1v1;

% loop through and calculate covariance for each
% transfered feature point
for ii=1:N;
  % covariance of ptxfer parameters assuming
  % x_21, u1, v1, and Z1 are uncorrelated
  Cov_p(9,9) = Cov_Z1(ii);

  % total Jacobian w.r.t. parameters
  J = [Ju_p(ii,:), Ju_f(ii,:);
       Jv_p(ii,:), Jv_f(ii,:)];
  
  % calculate 1st order covariance of predicted feature point (u2,v2)
  Cov_u2v2(:,:,ii) = J*Cov_p*J';
end;


%*******************************************************************************
function [Ju_p,Jv_p] = jacobian_wrt_pose(x_21,u1,v1,Z1,K);
% Ju_p is the Jacobian of u2 w.r.t pose vector x_21
% Jv_p is the Jacobian of v2 w.r.t pose vector x_21  

% preallocate Jacobian  
N = length(u1);  
Ju_p = zeros(N,6); 
Jv_p = zeros(N,6);

% vector of deltas to calculate J
% Hartley's suggestion for delta size
delta = max(abs(1e-4*x_21),1e-6);

% numerically evaluate the Jacobian
for ii = 1:6;
  pfwd = x_21;                              % initialize pfwd
  pbwd = x_21;                              % initialize pbwd  
  pfwd(ii) = pfwd(ii) + delta(ii);          % perturb p along dimension ii
  pbwd(ii) = pbwd(ii) - delta(ii);          % perturb p along dimension ii  
  [u2fwd,v2fwd] = ptxfer(pfwd,u1,v1,Z1,K);  % evaluate pfwd
  [u2bwd,v2bwd] = ptxfer(pbwd,u1,v1,Z1,K);  % evaluate pbwd  
  Ju_p(:,ii) = (u2fwd-u2bwd)/(2*delta(ii)); % calculate Jacobian
  Jv_p(:,ii) = (v2fwd-v2bwd)/(2*delta(ii));
end;


%*******************************************************************************
function [Ju_f,Jv_f] = jacobian_wrt_points(x_21,u1,v1,Z1,K);
% Ju_f is the Jacobian of u2 w.r.t. feature location (u1,v1,Z1)
% Jv_f is the Jacobian of v2 w.r.t. feature location (u1,v1,Z1)

% preallocate Jacobian
N = length(u1);
Ju_f = zeros(N,3);
Jv_f = zeros(N,3);

% effect of changing u1
delta = max(abs(1e-4*u1),1e-6);
[u2fwd,v2fwd] = ptxfer(x_21,u1+delta,v1,Z1,K);
[u2bwd,v2bwd] = ptxfer(x_21,u1-delta,v1,Z1,K);
Ju_f(:,1) = (u2fwd-u2bwd)./(2*delta);
Jv_f(:,1) = (v2fwd-v2bwd)./(2*delta);

% effect of changing v1
delta = max(abs(1e-4*v1),1e-6);
[u2fwd,v2fwd] = ptxfer(x_21,u1,v1+delta,Z1,K);
[u2bwd,v2bwd] = ptxfer(x_21,u1,v1-delta,Z1,K);
Ju_f(:,2) = (u2fwd-u2bwd)./(2*delta);
Jv_f(:,2) = (v2fwd-v2bwd)./(2*delta);

% effect of changing Z1
delat = max(abs(1e-4*Z1),1e-6);
[u2fwd,v2fwd] = ptxfer(x_21,u1,v1,Z1+delta,K);
[u2bwd,v2bwd] = ptxfer(x_21,u1,v1,Z1-delta,K);
Ju_f(:,3) = (u2fwd-u2bwd)./(2*delta);
Jv_f(:,3) = (v2fwd-v2bwd)./(2*delta);


%*******************************************************************************
function [u2,v2] = ptxfer(x_21,u1,v1,Z1,K);

t   = x_21(1:3);   % t21_2  explicit notation
rph = x_21(4:6);   % rph_21 explicit notation
R   = rotxyz(rph); % R21    explicit notation

U1  = homogenize(u1,v1);

%=================================================================
% If the depth Z to each point was known exactly, as measured along
% the optical axis in camera 1's coordinate frame, then the
% homogenous relation which transfers points from image 1 to image 2
% is:  x2 = K*R*inv(K)*x1 + K*t/Z
%=================================================================

%===================================================
% transfer points forward from image 1 into image 2
%===================================================
% homography at infinity
Hinf = K*R*inv(K);

% point transfer
U2 = Hinf*U1 + repmat(K*t,[1 size(U1,2)]) ./ repmat(Z1',[3 1]);

[u2,v2] = dehomogenize(U2);
