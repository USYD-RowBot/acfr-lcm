function tform = make_rad_tform(a,K,kr,kt);
%MAKE_RAD_TFORM  Returns a radial distortion compensation tform.
%   T = MAKE_RAD_TFORM(A,K,KR,KT) returns a Matlab tform structure,
%   T, which when used with imtransform *applies* the radial
%   distortion model to an undistorted image.  "A" is an [8x1]
%   vector of inverse distortion parameters as defined in the
%   Heikkila distortion model and as computed by the function
%   RADCOMP_OULU.  K is the [3x3] camera calibration matrix, kr is
%   a [3x1] vector of radial distortion coefficients, and kt is a
%   [2x1] vector of tangential distortion coefficients.
%
%   NOTE: To *compensate* images for radial distortion, use the
%   matlab function FLIPTFORM on the tform structure T.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-14-2002      rme         Created and written.


tdata.a  = a;
tdata.K  = K;
tdata.kr = kr;
tdata.kt = kt;

tform = maketform('custom',2,2,@radial_fwd,@radial_inv,tdata);



%=================================================================
function Ud = radial_fwd(Un,t)
% this function applies the forward radial distortion model
% i.e. it takes undistorted image coordinates and then distorts
% them according to the camera model


K = t.tdata.K;

% convert to homogenous represenation
Un(:,3) = ones(size(Un(:,1)));

% convert to form suitable for PRE multiply
% matrices as opposed to POST multiply matrices
Un = Un';

% convert to normalized image plane using camera
% calibration matrix
Xn = K^-1*Un;

% vectorize points
xn = Xn(1,:)';
yn = Xn(2,:)';

% apply heikkila forward distortion model
[xd,yd] = oulu_forward_distortion(t.tdata.kr,t.tdata.kt,xn,yn);

% map distorted normalized points back to image space (pixels)
Xd = [xd'; yd'; ones(size(xd'))];
Ud = K*Xd;

% dehomogenize
Ud = Ud(1:2,:);

% convert back to a form suitable for POST multiply
% matrices as opposed to PRE multiply matrices
Ud = Ud';


%=================================================================
function Un = radial_inv(Ud,t)
% this function compensates for radial distortion
% i.e. it takes as input distorted (observable) pixel coordinates
% and corrects for radial distortion to compute the ideal
% distortion-free image coordinates

K = t.tdata.K;

% convert to homogenous represenation
Ud(:,3) = ones(size(Ud(:,1)));

% convert to form suitable for PRE multiply
% matrices as opposed to POST multiply matrices
Ud = Ud';

% use camera calibration matrix to map coordinates
% to normalized image plane
Xd = K^-1*Ud;
xd = Xd(1,:)';
yd = Xd(2,:)';

% apply heikkila inverse distortion model
[xn,yn] = oulu_inverse_distortion(t.tdata.a,xd,yd);

% use camera calibration matrix to map normalized
% coordinates back to image space (pixels)
Xn = [xn'; yn'; ones(size(xn'))];
Un = K*Xn;

% dehomogenize
Un = Un(1:2,:);

% convert back to a form suitable for POST multiply
% matrices as opposed to PRE multiply matrices
Un = Un';
