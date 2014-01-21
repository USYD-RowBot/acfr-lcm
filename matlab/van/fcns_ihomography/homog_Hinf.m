function Hinf = homog_Hinf(K,R)
% homog_Hinf determines the homography of the plane at infinity.
%  
%   Hinf = homog_Hinf(K,R) returns the [3x3] infinite homography Hinf
%   (i.e. Hinf = K*R*inv(K)).  If R is the [3x3] rotation matrix between two
%   cameras who share the same optical center, then Hinf provides a mapping
%   of image points between the two cameras.  K is the [3x3] camera
%   calibration matrix.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-27-2003      rme         Created and written.

Hinf = K*R*inv(K);
