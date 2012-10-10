function [K,R,C] = krc(P)
% KRC   Camera matrix decomposition.
%    [K,R,C] = KRC(P) decomposes the [3x4] camera matrix P into a
%    [3x3] upper triangular matrix K, a [3x3] unitary matrix R, and
%    a [4x1] homogenous vector C.  
%
%    For a finite camera, (i.e. M, the left [3x3] submatrix of P,
%    is non-singular), the camera  matrix P can be written as:
%    P = M*[I | M^-1*p4] = K*R*[I | -C]
%    note that here C represents the [3x1] inhomogenous camera center
%
%    Note: in this implementation, the decomposition is chosen
%    such that all of the diagonal entries of K are positive.
%    Also, as explained in Hartley & Zisserman, pg 151, this
%    derivation of the camera model and its parameterization
%    assumes a right handed coordinate system in both image and
%    3D world, (i.e. y-axis is increases in the upwards direction
%    in the image plane).
%
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-08-2002      rme         Created and written.


M = P(1:3,1:3);
p1 = P(:,1);
p2 = P(:,2);
p3 = P(:,3);
p4 = P(:,4);

if rank(M) == 3
  % finite camera
  C = [-M^-1*p4;1];
  [K,R] = rq(M);
  % normalize so that the K(3,3) is 1
  K = K/K(3,3);
else
  % general projective camera
end


function [Rp,Qp] = rq(A)
%===============================================================
% The RQ decomposition is built upon MATLAB'S existing QR
% decomposition.  This is accomplished by making use of the 
% following permutation technique and is described in:
%
% N. Gracias, J. Santos-Victor, "Trajectory Reconstruction using
% Mosaic Registration", In Proceedings of SIRS'99, 1999.
%
% RQ decomposition of A is obtained from QR decomposition of A'E
% where E = [0 0 1; 0 1 0; 1 0 0];
% [Q,R] = qr(A'E);
%
% A = Rp*Qp where Rp = E*R'E, Qp = E*Q'
%===============================================================

E = [0 0 1; 0 1 0; 1 0 0];

[Q,R] = qr(A'*E);

% choose the decomposition for which the diagonal entries of
% R are all positive.  determine the appropriate unitary matrix
% Qxyz to accomplish this.
% for example, suppose that the (1,1) and (3,3) entries of R are
% both negative, then r = r<0 produces r = [1 0 1]'.
% this implies we want Qxyz to be: Qxyz = [-1 0 0; 0 1 0; 0 0 -1]
% so that multiplication by Qxyz (i.e. Qxyz*R) makes the (1,1)
% and (3,3) entries positive.
% (note that Qxyz = diag(-r+~r) = diag([-1 1 -1]') in this example)
r = diag(R);
r = r<0;
Qxyz = diag(-r+~r);

% QR decomposition produces
% A'*E = Q*R
% Q'*A'*E = R
% Qxyz*Q'*A'*E = Rn where diagonal entries are positive

% the *positive* upper diagonal QR decomposition then is given by:
Rn = Qxyz*R;
Qn = Q*Qxyz';

% finally, the RQ composition is given by:
Rp = E*Rn'*E;
Qp = E*Qn';
