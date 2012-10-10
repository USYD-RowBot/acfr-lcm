function [X_prime,Jminus] = inverse(X)
%INVERSE  6 DOF coordinate frame inverse relationship.
%   [X_ji,J_MINUS] = INVERSE(X_ij) returns the inverse coordinate
%   frame relationship of i w.r.t. j such that:
%   X_ij is the 6 DOF representation of frame j w.r.t. frame i.
%   X_ji is the 6 DOF representation of frame i w.r.t. frame j.
%   J_MINUS is the Jacobian of the inverse operation
%   i.e. J_MIUS = d(X_ji)/d(X_ij) evaluated at X_ij.
%
%   X_ji = (-)X_ij
%   X_ji = [x,y,z,r,p,h]'
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
%   Ex:  express local-level w.r.t. vehicle frame
%   % x_lv is the vehicle pose in the local-level frame with covariance P_lv
%   x_lv = rand(6,1);
%   P_lv = rand(6);
%
%   % x_vl is the pose of the local-level in the vehicle frame and 
%   % P_vl is the first order covariance
%   [x_vl,Jminus] = inverse(x_lv);
%   P_vl = Jminus*P_lv*Jminus';
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.



x = X(1);
y = X(2);
z = X(3);
r = X(4);
p = X(5);
h = X(6);
R = rotxyz([r,p,h]);

x_prime = -(R(1,1)*x + R(2,1)*y + R(3,1)*z);
y_prime = -(R(1,2)*x + R(2,2)*y + R(3,2)*z);
z_prime = -(R(1,3)*x + R(2,3)*y + R(3,3)*z);
h_prime = atan2(R(1,2), R(1,1));
p_prime = atan2(-R(1,3), R(1,1)*cos(h_prime) + R(1,2)*sin(h_prime));
r_prime = atan2(R(3,1)*sin(h_prime) - R(3,2)*cos(h_prime), ...
		-R(2,1)*sin(h_prime) + R(2,2)*cos(h_prime));

X_prime = [x_prime; y_prime; z_prime; r_prime; p_prime; h_prime];

% compute Jacobian
if nargout > 1
  % N
  N      =  zeros(3);
  N(1,1) =  0;
  N(1,2) = -R(3,1)*x*cos(h)-R(3,1)*y*sin(h)+z*cos(p);
  N(1,3) =  R(2,1)*x-R(1,1)*y;
  N(2,1) =  z_prime;
  N(2,2) = -R(3,2)*x*cos(h)-R(3,2)*y*sin(h)+z*sin(p)*sin(r);
  N(2,3) =  R(2,2)*x-R(1,2)*y;
  N(3,1) = -y_prime;
  N(3,2) = -R(3,3)*x*cos(h)-R(3,3)*y*sin(h)+z*sin(p)*cos(r);
  N(3,3) =  R(2,3)*x-R(1,3)*y;

  % Q
  Q      =  zeros(3);
  Q(1,1) = -R(1,1)/(1-R(1,3)^2);
  Q(1,2) = -R(1,2)*cos(r)/(1-R(1,3)^2);
  Q(1,3) =  R(3,3)*R(1,3)/(1-R(1,3)^2);
  Q(2,1) =  R(1,2)/(1-R(1,3)^2)^0.5;
  Q(2,2) = -R(3,3)*cos(h)/(1-R(1,3)^2)^0.5;
  Q(2,3) =  R(2,3)/(1-R(1,3)^2)^0.5;
  Q(3,1) =  R(1,1)*R(1,3)/(1-R(1,3)^2);
  Q(3,2) = -R(2,3)*cos(h)/(1-R(1,3)^2);
  Q(3,3) = -R(3,3)/(1-R(1,3)^2);

  % Jacobian
  Jminus = [-R',        N;
	     zeros(3),  Q];
end
