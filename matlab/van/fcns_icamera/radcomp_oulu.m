function [a,varargout] = radcomp_oulu(K,kr,kt,isize)
%RADCOMP_OULU  Computes the inverse radial distortion parameters.
%  A = RADCOMP_OULU(K,KR,KT,[NR NC]) computes the [8x1] parameter
%  vector A which is used in the Heikkila model to compensate for
%  radial distortion.  K is the [3x3] camera calibration matrix, KR
%  is the [3x1] vector of radial distortion coefficients, KT is the
%  [2x1] vector of tangential distortion coefficients, and NR,NC
%  are the number of image rows and columns respectively.
%
%  [A,UN,VN] = RADCOMP_OULU(K,KR,KT,[NR NC]) also returns [NRxNC]
%  matrices UN and VN which are the undistorted image coordinates
%  in pixels.
%
%  Reference: jheikkila-97a
%  Janne Heikkila, Olli Silven.  "A Four-step Camera Calibration
%  Procedure with Implicit Image Correction", University of Oulu,
%  Finland, 1997
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-14-2002      rme         Created and written.

nr = isize(1);
nc = isize(2);

%==================================================
% COMPUTE APPROX BOUNDARY OF NORMALIZED IMAGE PLANE
%==================================================
% image corners (pixels)
img_corners = [0,  nc-1; ...
	       0,  nr-1; ...
	       1,     1];

% normalized corners
norm_corners = K^-1*img_corners;


%=====================================================
% GENERATE A NxN DENSE GRID OF NORMALIZED IMAGE POINTS
%=====================================================
% ensure that the equally spaced grid of NxN tie-points
% covers the entire image and a small portion outside the
% effective area so that we can guarantee good results also
% for the border regions
N = 40;
dx = (norm_corners(1,2)-norm_corners(1,1))/nc;
dy = (norm_corners(2,2)-norm_corners(2,1))/nr;
[xn,yn] = meshgrid(linspace(norm_corners(1,1)-5*dx,norm_corners(1,2)+5*dx,N), ...
		   linspace(norm_corners(2,1)-5*dy,norm_corners(2,2)+5*dy,N));

% vectorize points
xn = xn(:);
yn = yn(:);


%==================================================
% APPLY DISTORTION MODEL TO NORMALIZED GRID POINTS
%==================================================
[xd,yd] = oulu_forward_distortion(kr,kt,xn,yn);


%===========================================================
% DETERMINE INVERSE MAPPING PARAMETERS 
% p = [a1,a2,a3,a4,a5,a6,a7,a8]'
% FOR INVERSE DISTORTION MODEL AS DESCRIBED IN jheikkila-97a
%===========================================================
r2 = xd.^2 + yd.^2;
r4 = r2.*r2;
r6 = r4.*r2;

u = [ -xd.*r2,    -xd.*r4,     -2*xd.*yd, -(r2+2*xd.^2), ...
       xn.*r4, xn.*xd.*r2,    xn.*yd.*r2,        xn.*r2 ];
v = [ -yd.*r2,    -yd.*r4, -(r2+2*yd.^2),     -2*xd.*yd, ...
       yn.*r4, yn.*xd.*r2,    yn.*yd.*r2,        yn.*r2 ];

T = zeros(N*N*2,8);
T(1:2:end-1,:) = u;
T(2:2:end,:)   = v;

e = zeros(N*N*2,1);
e(1:2:end-1) = xd-xn;
e(2:2:end)   = yd-yn;

% model parameters are estimated in a least-squares sense
% e = T*a
a = T\e;



%=======================================================
% USE THE HEIKKILA INVERSE DISTORTION MODEL WITH
% THE COMPUTED PARAMETERS TO COMPUTE THE UNDISTORTED
% IMAGE COORDINATES
%=======================================================
% check to see if function was called output arguments
% for the undistorted image coordinates, otherwise
% don't bother calculating them
if nargout == 3
  % distorted (observable) image coordinates
  [ud,vd] = meshgrid([0:nc-1],[0:nr-1]);
  ud = ud(:);
  vd = vd(:);

  % use camera calibration matrix to map coordinates
  % to normalized image plane
  Ud = [ud'; vd'; ones(size(ud'))];
  Xd = K^-1*Ud;
  xd = Xd(1,:)';
  yd = Xd(2,:)';
  
  % apply heikkila inverse distortion model
  [xn,yn] = oulu_inverse_distortion(a,xd,yd);
  
  % use camera calibration matrix to map normalized
  % coordinates back to image space (pixels)
  Xn = [xn'; yn'; ones(size(xn'))];
  Un = K*Xn;
  un = Un(1,:)';
  vn = Un(2,:)';
  
  % reshape vector of points into image array
  un = reshape(un,[nr nc]);
  vn = reshape(vn,[nr nc]);
  
  varargout{1} = un;
  varargout{2} = vn;
end
