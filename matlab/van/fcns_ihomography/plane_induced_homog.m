function [H21,varargout] = plane_induced_homog(R,t,K1,K2,n,d)
%PLANE_INDUCED_HOMOG  Compute homography relating two camera views
%                     of a single world plane.
%   H21 = PLANE_INDUCED_HOMOG(R,T,K1,K2,N,D) returns the [3x3] homogenous
%   matrix H21 which maps [3x1] homogenous coordinates from image 1
%   into image 2.  R is the [3x3] rotation matrix and T is the
%   [3x1] translation vector describing the pose of camera 2
%   relative to camera 1.  N is the [3x1] outward (away from camera)
%   world plane normal as expressed in camera 1's coordinate
%   frame.  D is the perpendicular distance from the camera 1
%   origin to the plane.  K1 and K2 are the two [3x3] linear camera
%   calibration matrices respectively.
%
%   [H21,TFORM] = PLANE_INDUCED_HOMOG_R,T,K1,K2,N,D) optionally
%   returns a Matlab tfrom structure.
%
%   The above assumptions imply that the two camera projection
%   matrices are:   P1 = K1*[I | 0]   P2 = K2*[R | T]
%   And the plane equation as expressed in camera 1's coordinate
%   frame is: N'*X_c1 = D
%
%   Reference:
%   [1] Hartley and Zisserman, Chp 12, pg. 313
%   [2] Bill Triggs, "Autocalibration from Planar Scenes", btriggs-98a
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-03-2002      rme         Created and written.
%    12-04-2002      rme         Modified to be consisent with
%                                Triggs reference.

% note: (+) sign in homography expression is opposite of formula shown in
% Hartley/Zisserman.  The below expression agrees with Triggs
% reference which defines an *outward* plane normal using the
% plane equation N'*X_c1 = D.  Hartley/Zisserman use a different
% plane equation, N'*X_c1 + D = 0, which explains the sign discrepency.
H21 = K2*(R + t*n'/d)*K1^-1;

if nargout == 2
  % note matlab assumes a post-multiply instead of a pre-multiply matrix
  varargout{1} = maketform('projective',H21');
end
