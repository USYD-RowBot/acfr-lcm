function [u2,v2,Cov_u2v2] = relview_ptxfer2(K,x_21,Z1,u1,v1,Cov_pose,Cov_Z1,Cov_u1v1);
%function [u2,v2,Cov_u2v2]= relview_ptxfer_sym(K,x_21,Z1,u1,v1,Cov_pose,Cov_Z1,Cov_u1v1);
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
  
if ~exist('Cov_u1v1','var') || isempty(Cov_u1v1);
  Cov_u1v1 = 2*eye(2); % assume sqrt(2) pixel sigma
end;

%==========================================================
% SETUP TO USE SIGMA POINT TRANSFER IN A VECTORIZED MANNER
%==========================================================
mu_x = [ x_21; ...
	    0;    % zero-mean placeholder for Z1
	 [0;0] ]; % zero-mean placeholder for u1,v1

P_xx = blkdiag(Cov_pose,Cov_Z1,Cov_u1v1);

%================================================================
% COMPUTE POINT TRANSFER BASED UPON SCENE DEPTH AND RELATIVE POSE
%================================================================
[mu_y,P_yy] = sigmaTransferVectorized(@ptxfer_wrapper,mu_x,P_xx,[],u1,v1,Z1,K);

u2 = mu_y(1,:)';
v2 = mu_y(2,:)';

Cov_u2v2 = squeeze(P_yy);

%===============================================================================
function Y = ptxfer_wrapper(X,u1,v1,Z1,K);

% extract sigma point elements
x_21 = X(1:6);
Z1   = Z1 + X(7); % add sigma point to Z1
u1   = u1 + X(8); % add sigma point to u1
v1   = v1 + X(9); % add sigma point to v1

% evaluate model
[u2,v2] = ptxfer(x_21,u1,v1,Z1,K);

% format output
Y = [u2'; v2'];

%===============================================================================
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
