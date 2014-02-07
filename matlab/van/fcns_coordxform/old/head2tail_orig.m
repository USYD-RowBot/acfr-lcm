function [X_3,Jplus] = head2tail(X_1,X_2)
%HEAD2TAIL  6 DOF coordinate frame composition.
%   [X_ik,J_PLUS] = HEAD2TAIL(X_ij,X_jk) returns the coordinate
%   frame composition of k to j and j to i such that:
%   X_jk is the 6 DOF representation of frame k w.r.t. frame j.
%   X_ij is the 6 DOF representation of frame j w.r.t. frame i.
%   X_ik is the 6 DOF representation of frame k w.r.t. frame i.
%   J_PLUS is the Jacobian of the composition operation
%   i.e. J_PLUS = d(X_ik)/d(X_ij,X_jk) evaluated at X_ij, X_jk.
%
%   X_ik = X_ij (+) X_jk
%   X_ik = [x,y,z,r,p,h]'
%
%   The above notation and associated Jacobian are based upon*:
%   R. Smith, M. Self, and P. Cheeseman.  "Estimating Uncertain
%   Spatial Relationships in Robotics".
%   
%   *Note: my XYZ Euler angle convention follows Fossen's convention
%   which is different than the SSC's.  My RPH are HPR in SSC's
%   notation, therefore the Jacobian I return is a permutation of the
%   Jacobian given in the SSC appendix.
%
%   Ex:  transformation from vehicle pose to camera pose
%   % x_lv is the vehicle pose in the local-level frame with covariance P_lv
%   x_lv = rand(6,1);
%   P_lv = rand(6);
%
%   % x_vc is the static pose of the camera in the vehicle frame 
%   % with covariance P_vc
%   x_vc = [1.4,0,0,0,0,pi/2]';
%   P_vc = zeros(6);
%
%   % x_lc is the pose of the camera in the local-level frame and 
%   % P_lc is the first order covariance
%   [x_lc,Jplus] = head2tail(x_lv,x_vc);
%   P_lc = Jplus*[P_lv, zeros(6); zeros(6), P_vc]*Jplus';
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.
%    11-15-2003      rme         Changed K_1(3,2) & K_2(3,2) to use tan(p_3)

% x_3 = x_1 (+) x_2
% x_3 = [x3, y3, z3, r3, p3, h3]'

% extract pose elements
x_1 = X_1(1); y_1 = X_1(2); z_1 = X_1(3);
r_1 = X_1(4); p_1 = X_1(5); h_1 = X_1(6);
R_1 = rotxyz([r_1,p_1,h_1]);

x_2 = X_2(1); y_2 = X_2(2); z_2 = X_2(3);
r_2 = X_2(4); p_2 = X_2(5); h_2 = X_2(6);
R_2 = rotxyz([r_2,p_2,h_2]);


% translation component
X_3 = zeros(6,1);
X_3(1:3) = R_1*[x_2; y_2; z_2] + [x_1; y_1; z_1];
x_3 = X_3(1); y_3 = X_3(2); z_3 = X_3(3);

% angle component
R_3 = R_1*R_2;
h_3 = atan2(R_3(2,1), R_3(1,1));
p_3 = atan2(-R_3(3,1), R_3(1,1)*cos(h_3) + R_3(2,1)*sin(h_3));
r_3 = atan2(R_3(1,3)*sin(h_3) - R_3(2,3)*cos(h_3), ...
	    -R_3(1,2)*sin(h_3) + R_3(2,2)*cos(h_3));
X_3(4) = r_3;
X_3(5) = p_3;
X_3(6) = h_3;

% compute the Jacobian
if nargout > 1
  % M
  M      =  zeros(3);
  M(1,1) =  R_1(1,3)*y_2 - R_1(1,2)*z_2;
  M(1,2) =  (z_3-z_1)*cos(h_1);
  M(1,3) = -(y_3-y_1);
  M(2,1) =  R_1(2,3)*y_2 - R_1(2,2)*z_2;
  M(2,2) =  (z_3-z_1)*sin(h_1);
  M(2,3) =  x_3-x_1;
  M(3,1) =  R_1(3,3)*y_2-R_1(3,2)*z_2;
  M(3,2) = -x_2*cos(p_1)-y_2*sin(p_1)*sin(r_1)-z_2*sin(p_1)*cos(r_1);
  M(3,3) =  0;

  % K_1
  K_1      =  zeros(3);
  K_1(1,1) =  [cos(p_1)*cos(h_3-h_1)]/cos(p_3);
  K_1(1,2) =  [sin(h_3-h_1)]/cos(p_3);
  K_1(1,3) =  0;
  K_1(2,1) = -cos(p_1)*sin(h_3-h_1);
  K_1(2,2) =  cos(h_3-h_1);
  K_1(2,3) =  0;
  K_1(3,1) =  [R_2(1,2)*sin(r_3) + R_2(1,3)*cos(r_3)]/cos(p_3);
  K_1(3,2) =  tan(p_3)*sin(h_3-h_1);
  K_1(3,3) =  1;

  % K_2
  K_2      =  zeros(3);
  K_2(1,1) =  1;
  K_2(1,2) =  tan(p_3)*sin(r_3-r_2);
  K_2(1,3) =  [R_1(1,3)*cos(h_3)+R_1(2,3)*sin(h_3)]/cos(p_3);
  K_2(2,1) =  0;
  K_2(2,2) =  cos(r_3-r_2);
  K_2(2,3) = -cos(p_2)*sin(r_3-r_2);
  K_2(3,1) =  0;
  K_2(3,2) =  sin(r_3-r_2)/cos(p_3);
  K_2(3,3) =  cos(p_2)*cos(r_3-r_2)/cos(p_3);

  % Jacobian
  Jplus = [eye(3),   M,   R_1,      zeros(3);
	   zeros(3), K_1, zeros(3), K_2];
end

