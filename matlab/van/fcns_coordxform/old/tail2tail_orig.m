function [X_jk,J] = tail2tail(X_ij,X_ik)
%TAIL2TAIL  6 DOF coordinate frame composition.
%   [X_jk,J] = TAIL2TAIL(X_ij,X_ik) returns the coordinate
%   frame composition of l to i and l to j such that:
%   X_ij is the 6 DOF representation of frame j w.r.t. frame i.
%   X_ik is the 6 DOF representation of frame k w.r.t. frame i.
%   X_jk is the 6 DOF representation of frame k w.r.t. frame j.
%   J is the Jacobian of the tail-to-tail operation
%   i.e. J = d(X_jk)/d(X_ij,X_ik) evaluated at X_ij, X_ik.
%
%   X_jk = (-)X_ij (+) X_ik
%   X_jk = [x,y,z,r,p,h]'
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
%   Ex:  relative pose of vehicle frame vj w.r.t vehicle frame vi
%   % x_lvi is the vehicle pose in the local-level frame at time t_i
%   % with covariance P_lvi
%   x_lvi = rand(6,1);
%   P_lvi = rand(6);
%
%   % x_lvj is the vehicle pose in the local-level frame at time t_j
%   % with covariance P_lvj
%   x_lvj = rand(6,1);
%   P_lvj = rand(6);
%
%   % x_vivj is the pose of the vehicle i w.r.t. vehicle frame j and 
%   % P_vivj is the first order covariance
%   [x_vivj,J] = tail2tail(x_lvi,x_lvj);
%   P_vivj = J*[P_lvi, zeros(6); zeros(6), P_lvj]*J';
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-10-2003      rme         Created and written.


[X_ji,Jminus] = inverse(X_ij);


[X_jk,Jplus] = head2tail(X_ji,X_ik);


J = [Jplus(:,1:6)*Jminus, Jplus(:,7:12)];
